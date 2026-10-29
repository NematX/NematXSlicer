///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Reader_DefaultGCodeReader_hpp_
#define slic3r_Plugins_GCode_Reader_DefaultGCodeReader_hpp_

#include "BuiltinGCodeReaderBase.hpp"

/*
Built-in G-code reader registration
===================================

The default provider supplies the conservative fallback used when probing does
not identify a firmware dialect. It inherits all supported commands from the
common parser and deliberately adds none: an extension it cannot model must
invalidate portable state instead of being guessed harmless.

    PluginLoader -> register_builtin_gcode_reader_plugins()
                 -> Orchestrator stores all eight providers
                 -> firmware requests its dialect Reader
                 -> read_gcode() validates and executes that exact provider

The registered object is process-lived and stateless. Per-stream parser state
is created on the stack for each run, allowing concurrent Reader executions.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

class DefaultGCodeReader final : public BuiltinGCodeReaderBase
{
public:
    explicit DefaultGCodeReader(orchestrator_handle *orchestrator);

private:
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    uint32_t probe_impl(std::string_view prefix) const override;
};

// Register the default, four Marlin-family, RepRapFirmware, Klipper and Bambu
// providers. The Orchestrator borrows the process-lived static plugin
// instances; this function transfers no ownership. Readers do not join an
// exclusive group because the host probes all active dialects and executes
// explicit ids independently.
void register_builtin_gcode_reader_plugins(orchestrator_handle *orchestrator);

}}} // namespace slic3r_api::GCodeGeneration::Reader

#endif // slic3r_Plugins_GCode_Reader_DefaultGCodeReader_hpp_
