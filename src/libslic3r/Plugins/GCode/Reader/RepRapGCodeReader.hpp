///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Reader_RepRapGCodeReader_hpp_
#define slic3r_Plugins_GCode_Reader_RepRapGCodeReader_hpp_

#include "libslic3r/Api/plugin/cpp/gcode/BuiltinGCodeReaderBase.hpp"

/*
RepRapFirmware G-code reader
============================

RepRapFirmware reuses several standard command numbers with distinct parameter
semantics. Its dispatchers call the common base first, then route only RRF
additions to virtual handlers. Overrides such as M106/M107 replace the common
handler directly, so the command is still dispatched exactly once.

Command coverage
----------------

Fully parsed into portable and/or private state:
  G10/G11, M106/M107, M116, M201, M203, M204, M205, M563, M566,
  M568, M572 and T-1, plus the common movement and modal commands.

Recognized without a represented state change:
  G4, M900 and M906.

Not parsed:
  RepRapFirmware expressions, macros, text-valued peripheral configuration,
  remote-board dotted device identifiers and commands not listed above. Such
  commands invalidate known state because their effects cannot be bounded.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

class RepRapGCodeReaderState;

class RepRapGCodeReader : public BuiltinGCodeReaderBase
{
public:
    explicit RepRapGCodeReader(orchestrator_handle *orchestrator);

protected:
    // Chain the parent textual dispatcher before adding RepRapFirmware T-1.
    bool consume_text_command(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              const GCodeLineParser::GCodeLine &line) const override;
    // Chain all portable commands first, then load private RRF state only for
    // an additional command which this class is about to process.
    bool consume_numeric_command(const plugin_run_context &run_context,
                                 const run_ctx_gcode_reader &context,
                                 GCodeStateTracker &state,
                                 const GCodeLineParser::GCodeLine &line,
                                 char letter,
                                 uint32_t number) const override;
    // RepRap command handlers are virtual so a more specialized RRF dialect
    // can replace one command without copying the numeric dispatcher. Each
    // handler owns the complete command and publishes its event before return.
    // Interpret bare G10 as firmware retract, or parameterized G10 as tool
    // temperature/standby/offset configuration.
    virtual void process_G10(const plugin_run_context &run_context,
                             const run_ctx_gcode_reader &context,
                             GCodeStateTracker &state,
                             RepRapGCodeReaderState &private_state,
                             const GCodeLineParser::GCodeLine &line) const;
    // Clear the private retracted state of the currently active tool.
    virtual void process_G11(const plugin_run_context &run_context,
                             const run_ctx_gcode_reader &context,
                             GCodeStateTracker &state,
                             RepRapGCodeReaderState &private_state) const;
    // Publish a wait event for one requested tool or all known tool targets.
    virtual void process_M116(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              const GCodeLineParser::GCodeLine &line) const;
    // Resolve omitted P through the active tool's private fan mapping and use
    // RepRapFirmware's normalized 0..1 fan speed.
    void process_M106_M107(const plugin_run_context &run_context,
                           const run_ctx_gcode_reader &context,
                           GCodeStateTracker &state,
                           const GCodeLineParser::GCodeLine &line,
                           uint32_t number) const override;
    // Store per-axis maximum accelerations in the private RRF snapshot.
    virtual void process_M201(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              RepRapGCodeReaderState &private_state,
                              const GCodeLineParser::GCodeLine &line) const;
    // Store maximum and minimum feedrates after converting mm/min to mm/s.
    virtual void process_M203(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              RepRapGCodeReaderState &private_state,
                              const GCodeLineParser::GCodeLine &line) const;
    // Update distinct print/travel acceleration registers and publish a
    // portable effective acceleration only when the command determines one.
    virtual void process_M204(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              RepRapGCodeReaderState &private_state,
                              const GCodeLineParser::GCodeLine &line) const;
    // Store the job-level jerk limits expressed by RepRapFirmware M205.
    virtual void process_M205(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              RepRapGCodeReaderState &private_state,
                              const GCodeLineParser::GCodeLine &line) const;
    // Replace one tool's drive, heater and fan mappings. Device lists must be
    // complete non-negative colon-separated identifiers.
    virtual void process_M563(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              RepRapGCodeReaderState &private_state,
                              const GCodeLineParser::GCodeLine &line) const;
    // Store machine jerk limits after converting mm/min to mm/s.
    virtual void process_M566(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              RepRapGCodeReaderState &private_state,
                              const GCodeLineParser::GCodeLine &line) const;
    // Update active and standby temperatures for one logical RRF tool.
    virtual void process_M568(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              RepRapGCodeReaderState &private_state,
                              const GCodeLineParser::GCodeLine &line) const;
    // Store pressure advance for the drive selected by D.
    virtual void process_M572(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              RepRapGCodeReaderState &private_state,
                              const GCodeLineParser::GCodeLine &line) const;
    // Represent T-1 by invalidating active-tool knowledge; zero would mean T0
    // and therefore cannot encode the deselected state.
    virtual void process_T_minus_1(const plugin_run_context &run_context,
                                  const run_ctx_gcode_reader &context,
                                  GCodeStateTracker &state) const;

private:
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    uint32_t probe_impl(std::string_view prefix) const override;
    bool is_neutral_command(char letter, uint32_t number) const override;
    bool has_unsupported_state_effect(char letter,
                                      uint32_t number) const override;
    std::optional<raw_gcode_reader_marker_type> marker_type(
        char letter, uint32_t number) const override;
    void validate_private_state(const GCodeStateTracker &state) const override;
    uint32_t reader_state_id_impl() const override { return m_reader_state_id; }

    uint32_t m_reader_state_id;
};

}}} // namespace slic3r_api::GCodeGeneration::Reader

#endif // slic3r_Plugins_GCode_Reader_RepRapGCodeReader_hpp_
