#include "libslic3r/Api/plugin/cpp/gcode/BuiltinGCodeReaderBase.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/DefaultGCodeFirmwareSession.hpp"

/*
The isolated SDK DLL also registers a reader derived from the shared stream
parser. Loading this library must resolve the reader's complete virtual table
without host C++ headers. Registration borrows the real orchestrator and keeps
the provider alive in this module for as long as the loader retains the DLL.
The registration log confirms both the firmware probe and reader registration,
so a CLI startup can check this fixture through the production package loader.

    register_plugin()
    |-- gcode_sdk_probe(): check the linked firmware implementation
    |-- ExternalReader(): construct a public-SDK parser
    `-- orchestrator_register_plugin(): lend its function table to the host
*/

// Exercise the firmware probe implemented in the other SDK-only translation unit.
extern "C" int gcode_sdk_probe();

namespace {
class ExternalReader final : public slic3r_api::GCodeGeneration::Reader::BuiltinGCodeReaderBase
{
public:
    // Bind the provider to the real host supplied by the package loader.
    explicit ExternalReader(orchestrator_handle *orchestrator) : BuiltinGCodeReaderBase(orchestrator) {}
private:
    // Return the fixture identity as borrowed static text.
    const char *id_impl() const noexcept override { return "gcode.reader.sdk_only_probe"; }
    // Return the display name without retaining caller state.
    const char *name_impl() const noexcept override { return "SDK-only Reader probe"; }
    // Describe the linkage fixture rather than a product-specific dialect.
    const char *description_impl() const noexcept override { return "Public SDK linkage test."; }
};
}

// A real provider entry point also forces the link of the complete reader
// vtable. Registration receives the real orchestrator, never a fabricated handle.
extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *orchestrator);

extern "C" SLIC3R_PLUGIN_API void register_plugin(orchestrator_handle *orchestrator)
{
    if (gcode_sdk_probe() != 0) {
        orchestrator_send_log(orchestrator, RAW_PLUGIN_LOG_ERROR, "SDK-only firmware probe failed.");
        return;
    }
    static ExternalReader reader(orchestrator);
    orchestrator_register_plugin(orchestrator, reader.c_instance());
    if (orchestrator_is_plugin_registered(orchestrator, "gcode.reader.sdk_only_probe") != 0)
        orchestrator_send_log(orchestrator, RAW_PLUGIN_LOG_INFO,
            "SDK-only firmware probe passed and reader registered.");
}

#include "libslic3r/Api/plugin/c/slic3r_plugin_register_version.h"
