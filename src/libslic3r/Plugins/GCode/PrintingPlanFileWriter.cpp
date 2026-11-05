///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "PrintingPlanFileWriter.hpp"

#include <cstdint>
#include <exception>
#include <string>
#include <string_view>

#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>
#include <boost/nowide/fstream.hpp>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode.h"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeFirmwareViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeWriterViews.hpp"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/Api/plugin/cpp/PrintingPlanViews.hpp"
#include "libslic3r/FilesystemTransaction.hpp"
#include "libslic3r/libslic3r.h"

/*
PrintingPlan file serializer
============================

This STEP_GCODE plugin (priority 0) creates the output artifact for the
selected firmware implementation. Plugins of higher priority run after it, find
the published file at the same path and may modify it before the host Reader
consumes it. It walks the already ordered PrintingPlan and writes the
chunks returned by the firmware session to one atomically published file. The
plugin does not interpret extrusion geometry or assemble individual machine
commands.

The normal execution flow is:

Every firmware result is written before the next firmware callback is called,
because the returned string may be backed by state owned by the session. Empty
groups and layers are still forwarded so firmware implementations can observe
their boundaries. The firmware view owns G-code generation and state changes;
this plugin only provides the traversal, stream checks, and publication
guarantee. A cleanup warning after a successful commit is reported separately
from an output-generation failure.

Operation notes:
- count all extrusion roots for progress reporting
- validate the G-code context and output path
- open a sibling temporary file
- serialize the nested plan boundaries in order:
- print begin/events/end
- group begin/events/end
- layer begin/events/end
- tool-group begin/events/end
- extrusion and event chunks
- flush and close the temporary stream
- replace the final output through FilesystemTransaction
- remove the temporary file if serialization fails

Representative direct calls (branches are not a timeline):

    write_plan_file()
        |-- serialize_plan()
        |-- publish_output_file()
        `-- slic3r_api::publish_gcode_reader_source_path()

    serialize_plan()
        |-- firmware.begin_print()
        |-- firmware.begin_group()
        |-- firmware.write_event()
        |-- firmware.begin_layer()
        |-- firmware.begin_tool_group()
        `-- firmware.write_extrusion()
*/

namespace slic3r_api { namespace GCodeGeneration { namespace PrintingPlanFileWriterPlugin {
namespace {

const char *const k_no_dependencies[] = { nullptr };

// Count the ordered work units once so progress stays stable while chunks are
// produced sequentially.
uint32_t count_plan_extrusions(const ConstPrintingPlan &plan,
                               const plugin_run_context *run_context);

// Append one borrowed firmware result before another firmware callback can
// invalidate its storage.
void write_firmware_chunk(boost::nowide::ofstream &stream,
                          const plugin_run_context *run_context,
                          std::string_view chunk);

// Traverse every plan boundary in strict nesting order and forward it to the
// stateful firmware session.
void serialize_plan(boost::nowide::ofstream &stream,
                    const plugin_run_context *run_context,
                    const Print &print,
                    const ConstPrintingPlan &plan,
                    const GCodeFirmwareView &firmware,
                    PluginProgress &progress);

// Replace the final output only after the staging stream is fully closed.
void publish_output_file(const boost::filesystem::path &temporary_path,
                         const boost::filesystem::path &final_path);

// Own the complete staging, serialization, stream validation, and publication
// lifecycle for one STEP_GCODE run.
void write_plan_file(const plugin_run_context &run_context,
                     const run_ctx_generate_gcode &context,
                     PluginProgress &progress);

uint32_t count_plan_extrusions(const ConstPrintingPlan &plan,
                               const plugin_run_context *run_context)
{
    uint32_t count = 0;
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        throw_if_cancelled(run_context);
        const ConstPrintingGroup group = plan.group(group_idx);
        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            throw_if_cancelled(run_context);
            const ConstPrintingLayerGroup layer = group.layer_group(layer_idx);
            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                throw_if_cancelled(run_context);
                count += layer.tool_group(tool_idx).extrusion_count();
            }
        }
    }
    return count;
}

void write_firmware_chunk(boost::nowide::ofstream &stream,
                          const plugin_run_context *run_context,
                          std::string_view chunk)
{
    throw_if_cancelled(run_context);
    if (chunk.empty())
        return;

    stream.write(chunk.data(), static_cast<std::streamsize>(chunk.size()));
    if (!stream)
        throw Slic3r::RuntimeError("The temporary G-code stream rejected firmware output.");
}

void serialize_plan(boost::nowide::ofstream &stream,
                    const plugin_run_context *run_context,
                    const Print &print,
                    const ConstPrintingPlan &plan,
                    const GCodeFirmwareView &firmware,
                    PluginProgress &progress)
{
    throw_if_cancelled(run_context);
    throw_if_cancelled(run_context);
    write_firmware_chunk(stream, run_context, firmware.begin_print(print, plan));
    const ConstPrintingScopeEvents plan_events = plan.events();
    // begin_print consumes the plan before-events as part of its preamble.

    // Every begin call is matched before the enclosing scope is left. Empty
    // groups and layers are still observable because a firmware may need their
    // boundaries even when they contain no extrusion.
    for (uint32_t group_idx = 0; group_idx < plan.group_count(); ++group_idx) {
        throw_if_cancelled(run_context);
        const ConstPrintingGroup group = plan.group(group_idx);
        throw_if_cancelled(run_context);
        write_firmware_chunk(stream, run_context, firmware.begin_group(group));
        const ConstPrintingScopeEvents group_events = group.events();
        if (group_events.has_before()) {
            throw_if_cancelled(run_context);
            write_firmware_chunk(stream, run_context, firmware.write_event(group_events.before()));
        }

        for (uint32_t layer_idx = 0; layer_idx < group.layer_group_count(); ++layer_idx) {
            throw_if_cancelled(run_context);
            const ConstPrintingLayerGroup layer = group.layer_group(layer_idx);
            throw_if_cancelled(run_context);
            write_firmware_chunk(stream, run_context, firmware.begin_layer(layer));
            const ConstPrintingScopeEvents layer_events = layer.events();
            if (layer_events.has_before()) {
                throw_if_cancelled(run_context);
                write_firmware_chunk(stream, run_context, firmware.write_event(layer_events.before()));
            }

            for (uint32_t tool_idx = 0; tool_idx < layer.tool_group_count(); ++tool_idx) {
                throw_if_cancelled(run_context);
                const ConstPrintingToolGroup tool_group = layer.tool_group(tool_idx);
                throw_if_cancelled(run_context);
                write_firmware_chunk(stream, run_context, firmware.begin_tool_group(tool_group));
                const ConstPrintingScopeEvents tool_events = tool_group.events();
                if (tool_events.has_before()) {
                    throw_if_cancelled(run_context);
                    write_firmware_chunk(stream, run_context, firmware.write_event(tool_events.before()));
                }

                for (uint32_t extrusion_idx = 0;
                     extrusion_idx < tool_group.extrusion_count();
                     ++extrusion_idx) {
                    throw_if_cancelled(run_context);
                    const ConstPrintingExtrusion extrusion = tool_group.extrusion(extrusion_idx);
                    write_firmware_chunk(stream, run_context, firmware.write_extrusion(extrusion));
                    progress.increment();
                }

                if (tool_events.has_after()) {
                    throw_if_cancelled(run_context);
                    write_firmware_chunk(stream, run_context, firmware.write_event(tool_events.after()));
                }
                throw_if_cancelled(run_context);
                write_firmware_chunk(stream, run_context, firmware.end_tool_group());
            }

            if (layer_events.has_after()) {
                throw_if_cancelled(run_context);
                write_firmware_chunk(stream, run_context, firmware.write_event(layer_events.after()));
            }
            throw_if_cancelled(run_context);
            write_firmware_chunk(stream, run_context, firmware.end_layer());
        }

        if (group_events.has_after()) {
            throw_if_cancelled(run_context);
            write_firmware_chunk(stream, run_context, firmware.write_event(group_events.after()));
        }
        throw_if_cancelled(run_context);
        write_firmware_chunk(stream, run_context, firmware.end_group());
    }

    if (plan_events.has_after()) {
        throw_if_cancelled(run_context);
        write_firmware_chunk(stream, run_context, firmware.write_event(plan_events.after()));
    }
    throw_if_cancelled(run_context);
    write_firmware_chunk(stream, run_context, firmware.end_print());
}

void publish_output_file(const boost::filesystem::path &temporary_path,
                         const boost::filesystem::path &final_path)
{
    Slic3r::FilesystemTransaction transaction;
    transaction.add_replacement(temporary_path, final_path);
    const Slic3r::FilesystemTransactionResult result = transaction.commit();

    // Cleanup warnings concern hidden transaction artifacts after the visible
    // output was committed. Report them without turning a valid file into a
    // failed export.
    for (const Slic3r::FilesystemTransactionFailure &warning : result.cleanup_warnings)
        BOOST_LOG_TRIVIAL(warning) << "G-code output cleanup warning: "
                                   << Slic3r::format_filesystem_transaction_failure(warning);

    if (result.status != Slic3r::FilesystemTransactionStatus::Committed)
        throw Slic3r::RuntimeError(
            "Could not publish the G-code output file: " +
            Slic3r::format_filesystem_transaction_error(result));
}

void write_plan_file(const plugin_run_context &run_context,
                     const run_ctx_generate_gcode &context,
                     PluginProgress &progress)
{
    throw_if_cancelled(&run_context);
    if (context.print == nullptr || context.plan == nullptr || context.firmware == nullptr ||
        context.output_path == nullptr || context.output_path[0] == '\0')
        throw Slic3r::RuntimeError("PrintingPlan file writer received an incomplete G-code context.");

    const boost::filesystem::path final_path(context.output_path);
    const boost::filesystem::path temporary_path = final_path.parent_path() /
        (final_path.filename().string() + ".tmp");
    boost::system::error_code cleanup_error;
    boost::filesystem::remove_all(temporary_path, cleanup_error);

    boost::nowide::ofstream stream(
        temporary_path.string().c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
    if (!stream)
        throw Slic3r::RuntimeError(
            "Cannot open temporary G-code output file '" + temporary_path.string() + "'.");

    try {
        const Print print(context.print);
        const ConstPrintingPlan plan(context.plan);
        const GCodeFirmwareView firmware(context.firmware);
        serialize_plan(stream, &run_context, print, plan, firmware, progress);

        // Individual writes may succeed while the operating system reports a
        // delayed error during flush or close. Validate both before publishing
        // the staging file.
        stream.flush();
        if (!stream)
            throw Slic3r::RuntimeError("Failed while flushing the temporary G-code output file.");
        stream.close();
        if (!stream)
            throw Slic3r::RuntimeError("Failed while closing the temporary G-code output file.");

        throw_if_cancelled(&run_context);
        publish_output_file(temporary_path, final_path);
        // The final text is now closed and atomically visible. Publish it only
        // after that point so the host Reader never observes partial output.
        slic3r_api::publish_gcode_reader_source_path(context, final_path.string());
    } catch (...) {
        try {
            stream.close();
        } catch (...) {
            // Cleanup is best effort; preserve the original serialization error.
        }
        boost::filesystem::remove_all(temporary_path, cleanup_error);
        throw;
    }
}

class PrintingPlanFileWriter final : public PluginBase
{
public:
    static PrintingPlanFileWriter &instance(orchestrator_handle *orchestrator)
    {
        static PrintingPlanFileWriter plugin(orchestrator);
        return plugin;
    }

    explicit PrintingPlanFileWriter(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

private:
    const char *id_impl() const noexcept override { return "gcode.printing_plan_file_writer"; }
    const char *name_impl() const noexcept override { return "PrintingPlan file writer"; }
    const char *description_impl() const noexcept override
    {
        return "Writes ordered firmware chunks to one atomically published output file.";
    }
    slicing_step_t step_impl() const noexcept override { return STEP_GCODE; }
    const char *const *dependencies_impl() const noexcept override { return k_no_dependencies; }
    // Priority 0 runs before plugins with a higher value; a plugin with a lower value
    // runs before this writer, before any fresh file exists.
    int32_t priority_impl() const noexcept override { return 0; }
    const char *progress_message_format_impl() const noexcept override
    {
        return "Writing G-code: %u / %u extrusions";
    }

    void setup_run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        const run_ctx_generate_gcode *context = plugin_ctx_as_generate_gcode(run_ctx);
        if (context != nullptr && context->plan != nullptr)
            progress().add_max(count_plan_extrusions(ConstPrintingPlan(context->plan), run_ctx));
    }

    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        const run_ctx_generate_gcode *context = plugin_ctx_as_generate_gcode(run_ctx);
        if (context == nullptr)
            throw Slic3r::RuntimeError("PrintingPlan file writer received the wrong step payload.");
        write_plan_file(*run_ctx, *context, progress());
    }
};

} // namespace

void register_printing_plan_file_writer_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(
        orchestrator, PrintingPlanFileWriter::instance(orchestrator).c_instance());
}

}}} // namespace slic3r_api::GCodeGeneration::PrintingPlanFileWriterPlugin
