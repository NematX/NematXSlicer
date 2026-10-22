///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "PostProcessScripts.hpp"

#include <string>

#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>

#include "libslic3r/Api/plugin/c/steps/slic3r_step_gcode.h"
#include "libslic3r/Api/plugin/cpp/PluginBase.hpp"
#include "libslic3r/FilesystemTransaction.hpp"
#include "libslic3r/GCode/PostProcessor.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/Print.hpp"

/*
Post-processing scripts
=======================

This STEP_GCODE plugin runs after the built-in writer (it depends on it and has
a higher priority). It finds the file the writer has just published at
`output_path` and lets the scripts of the `post_process` option modify it before
the host Reader consumes it. The viewer and the print statistics, which come from
that reading, therefore describe the post-processed text. The header, thumbnails
and configuration block that an exporter adds later are not part of the file the
scripts see.

The normal execution flow is:

The scripts never touch output_path itself: if one fails, the writer's file stays
as it was and the step fails with the message of the script. The plugin does not
publish a Reader source: the writer already published output_path, which remains
the file that is read.

The scripts read the configuration from the SLIC3R_* environment variables of the
process (see ConfigBase::setenv_()), taken from the complete print configuration of the Print.
Waiting for a script cannot be interrupted: the cancellation is only observed
before and after it.

Operation notes:
- check the cancellation and the payload
- run_post_process_scripts(): copy the file to "<output_path>.pp" and run the
- scripts on the copy; with no script configured nothing is read or written
- publish_processed_copy(): replace output_path by the copy transactionally
- check the cancellation again

Representative direct calls (branches are not a timeline):

    run_post_process()
        |-- throw_if_cancelled()
        |-- Slic3r::run_post_process_scripts()
        `-- publish_processed_copy()
*/

namespace slic3r_api { namespace GCodeGeneration { namespace PostProcessScriptsPlugin {
namespace {

const char *const k_writer_id = "gcode.printing_plan_file_writer";
const char *const k_dependencies[] = { k_writer_id, nullptr };

// Replace the published file by the processed copy. The copy is a sibling of the
// file; the transaction consumes it even when the replacement is rejected. If the
// publication fails, the transaction tries to restore the previous file; a failed
// publication or restoration throws and so fails the step.
void publish_processed_copy(const std::string &processed_path, const std::string &output_path);

// Run the scripts on the file published by the writer, then publish the result.
void run_post_process(const plugin_run_context &run_context, const run_ctx_generate_gcode &context);

void publish_processed_copy(const std::string &processed_path, const std::string &output_path)
{
    Slic3r::FilesystemTransaction transaction;
    transaction.add_replacement(processed_path, output_path);
    const Slic3r::FilesystemTransactionResult result = transaction.commit();

    // Cleanup warnings concern hidden artifacts after the visible file was replaced.
    for (const Slic3r::FilesystemTransactionFailure &warning : result.cleanup_warnings)
        BOOST_LOG_TRIVIAL(warning) << "Post-processed G-code cleanup warning: "
                                   << Slic3r::format_filesystem_transaction_failure(warning);

    if (result.status != Slic3r::FilesystemTransactionStatus::Committed)
        throw Slic3r::RuntimeError(
            "Could not publish the post-processed G-code: " +
            Slic3r::format_filesystem_transaction_error(result));
}

void run_post_process(const plugin_run_context &run_context, const run_ctx_generate_gcode &context)
{
    throw_if_cancelled(&run_context);
    if (context.print == nullptr || context.output_path == nullptr || context.output_path[0] == '\0')
        throw Slic3r::RuntimeError("The post-processing plugin received an incomplete G-code context.");

    // Built-in plugins of the library read the host Print behind the handle, like the infill generator does.
    const Slic3r::Print &print = *reinterpret_cast<const Slic3r::Print *>(context.print);
    const std::string output_path = context.output_path;
    const std::string processed_path = Slic3r::run_post_process_scripts(output_path, print.full_print_config());
    if (processed_path.empty())
        return;

    // The copy is handed to the transaction at once, so that no later check can leave it behind.
    publish_processed_copy(processed_path, output_path);
    throw_if_cancelled(&run_context);
}

class PostProcessScripts final : public PluginBase
{
public:
    static PostProcessScripts &instance(orchestrator_handle *orchestrator)
    {
        static PostProcessScripts plugin(orchestrator);
        return plugin;
    }

    explicit PostProcessScripts(orchestrator_handle *orchestrator) : PluginBase(orchestrator) {}

private:
    const char *id_impl() const noexcept override { return "gcode.post_process"; }
    const char *name_impl() const noexcept override { return "Post-processing scripts"; }
    const char *description_impl() const noexcept override
    {
        return "Runs the post-processing scripts on the generated G-code before it is read for the preview.";
    }
    slicing_step_t step_impl() const noexcept override { return STEP_GCODE; }
    // The writer creates the file this plugin modifies: the plugin is blocked without it and runs after it.
    const char *const *dependencies_impl() const noexcept override { return k_dependencies; }
    int32_t priority_impl() const noexcept override { return 100; }
    int32_t used_config_keys(raw_used_config_key *keys) const noexcept override
    {
        if (keys != nullptr)
            keys[0] = raw_used_config_key{"post_process", RAW_CO_VECTOR_STRING, RAW_CONTAINER_TYPE_NONE, RAW_PRESET_TYPE_NONE};
        return 1;
    }

    void run_impl(const plugin_run_context *run_ctx) const override
    {
        throw_if_cancelled(run_ctx);
        const run_ctx_generate_gcode *context = plugin_ctx_as_generate_gcode(run_ctx);
        if (context == nullptr)
            throw Slic3r::RuntimeError("The post-processing plugin received the wrong step payload.");
        run_post_process(*run_ctx, *context);
    }
};

} // namespace

void register_post_process_scripts_plugin(orchestrator_handle *orchestrator)
{
    orchestrator_register_plugin(
        orchestrator, PostProcessScripts::instance(orchestrator).c_instance());
}

}}} // namespace slic3r_api::GCodeGeneration::PostProcessScriptsPlugin
