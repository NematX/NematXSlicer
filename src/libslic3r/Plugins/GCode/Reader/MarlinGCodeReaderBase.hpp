///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Reader_MarlinGCodeReaderBase_hpp_
#define slic3r_Plugins_GCode_Reader_MarlinGCodeReaderBase_hpp_

#include "libslic3r/Api/plugin/cpp/gcode/BuiltinGCodeReaderBase.hpp"

/*
Common Marlin-family G-code reader
==================================

Marlin 1, Marlin 2, Prusa and Sprinter share most command syntax. This abstract
base keeps that common behavior in one place while leaving M204 to each
concrete dialect:

    BuiltinGCodeReaderBase
        `-- MarlinGCodeReaderBase
              |-- M109 R cooling waits
              |-- shared neutral commands and timeline markers
              `-- virtual process_M204()

The class is not registered as a plugin. A concrete Reader supplies its stable
plugin identity, probe and M204 interpretation. Parsing state belongs to one
read invocation, so process-lived Reader instances remain safe for concurrent
reads.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

class MarlinGCodeReaderBase : public BuiltinGCodeReaderBase
{
public:
    explicit MarlinGCodeReaderBase(orchestrator_handle *orchestrator);

protected:
    // Dispatch common commands first, then route M204 to the concrete
    // firmware dialect. True means that a handler published the command.
    bool consume_numeric_command(const plugin_run_context &run_context,
                                 const run_ctx_gcode_reader &context,
                                 GCodeStateTracker &state,
                                 const GCodeLineParser::GCodeLine &line,
                                 char letter,
                                 uint32_t number) const override;
    // Marlin accepts R as the M109 target when S is absent. Other temperature
    // forms retain the portable implementation.
    void process_M104_M109(const plugin_run_context &run_context,
                           const run_ctx_gcode_reader &context,
                           GCodeStateTracker &state,
                           const GCodeLineParser::GCodeLine &line,
                           uint32_t number) const override;
    // M204 has incompatible parameter semantics across this family. Every
    // concrete Reader must publish exactly one synchronous event for it.
    virtual void process_M204(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              const GCodeLineParser::GCodeLine &line) const = 0;
    bool is_neutral_command(char letter, uint32_t number) const override;
    bool has_unsupported_state_effect(char letter,
                                      uint32_t number) const override;
    std::optional<raw_gcode_reader_marker_type> marker_type(
        char letter, uint32_t number) const override;
};

}}} // namespace slic3r_api::GCodeGeneration::Reader

#endif // slic3r_Plugins_GCode_Reader_MarlinGCodeReaderBase_hpp_
