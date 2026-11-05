///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Reader_BambuGCodeReader_hpp_
#define slic3r_Plugins_GCode_Reader_BambuGCodeReader_hpp_

#include "Marlin2GCodeReader.hpp"

/*
Bambu Lab G-code reader
=======================

Bambu printers run a Marlin 2 derived firmware and understand the Marlin 2
movement, temperature and fan commands. `M106` / `M107` are the exception: Bambu
reads P as a fan number, not as a tool (see process_M106_M107() below). They also
receive many printer-specific commands from the slicer: AMS filament loading (`M620`,
`M621`), object labels (`M624`/`M625`), spaghetti detection (`M981`), power-loss
recovery (`M1003`), first-layer scan (`M976`), and so on.

None of those commands changes a value the Reader models (position, extrusion,
temperature, fan speed, tool...). Without this Reader, the Marlin 2 parser does
not know them: every one of them publishes an UNKNOWN_COMMAND diagnostic and
invalidates all the known state, so the segments following the command are
lost until a complete X/Y/Z position is read again.

    BuiltinGCodeReaderBase
        `-- MarlinGCodeReaderBase
              `-- Marlin2GCodeReader
                    `-- BambuGCodeReader
                          |-- is_neutral_command(): plain Bambu commands (M624, M981...)
                          |-- consume_text_command(): dotted sub-codes (M620.1, G29.2...)
                          |-- process_M106_M107(): P1 / no P, P2 and P3 fans
                          `-- probe_impl(): Bambu-only marker in the top comments

Fans: `M106 P1 S..`, `M106 S..` and `M107` set the part cooling fan of the active
tool (FAN_SPEED, target = the active tool). While no tool is known, the Marlin 2
meaning applies (P1: the fan of the tool 1; no P: the fan of the tool 0).
`M106 P2` is the auxiliary (platter) fan and `M106 P3` the exhaust fan: they are
published as the items of BambuGCodeReaderItems.hpp, never as FAN_SPEED, so they
cannot be read as the fans of the tools 2 and 3. `M107 P2` / `M107 P3` set them to
zero. Any other P (P0, P4...) keeps the Marlin 2 meaning: the fan of the tool P.

Two mechanisms are needed because the common parser reads a command number
with a strict integer parse:
  - `M624`, `M1002 gcode_claim_action : 1`... have an integer number, so they
    reach is_neutral_command() (the text after the command word is ignored);
  - `M620.1`, `G29.2`... are not numbers, so the parser offers them to
    consume_text_command() first. This Reader recognizes an exact list of those
    sub-codes there and publishes them like any other neutral command.

Deliberately left unknown (they do change the state, or its effect is not
known): `G29` (probing, final position unknown), `G29.1` (Z offset), `M620.1`
with an X, Y or Z parameter (toolhead path during an AMS filament change),
`M1020` and anything else not listed in the .cpp file. `T255`, `T1000` and
`T65535` stay plain tool selections, as in the Marlin 2 Reader.

The detection probe returns 0 unless a comment line before the first
executable command contains a marker only Bambu printers produce
(`BambuStudio` or `printer_model = Bambu Lab`). `OrcaSlicer` alone is not one:
Orca also writes it for Klipper printers. Without this rule the Reader would
tie with Marlin 2 for every ordinary Marlin file (see
Marlin2GCodeReader::probe_impl) and could be selected for it.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

class BambuGCodeReader final : public Marlin2GCodeReader
{
public:
    // Bind the Reader to the orchestrator and register the identity of the Bambu
    // private state (the fan items); a failed registration throws.
    explicit BambuGCodeReader(orchestrator_handle *orchestrator);

protected:
    // Chain the parent text dispatch, then consume the Bambu dotted sub-codes
    // (M620.1 without X/Y/Z, G29.2...) as neutral commands. True means the
    // command was published here.
    bool consume_text_command(const plugin_run_context &run_context,
                              const run_ctx_gcode_reader &context,
                              GCodeStateTracker &state,
                              const GCodeLineParser::GCodeLine &line) const override;
    // The Marlin 2 neutral commands plus the Bambu integer commands.
    bool is_neutral_command(char letter, uint32_t number) const override;
    // Publish P2 / P3 as the platter / exhaust fan items and P1 or no P as the
    // fan of the active tool; any other P is handled by the Marlin 2 parser.
    void process_M106_M107(const plugin_run_context &run_context,
                           const run_ctx_gcode_reader &context,
                           GCodeStateTracker &state,
                           const GCodeLineParser::GCodeLine &line,
                           uint32_t number) const override;

private:
    // Plugin identity: "gcode.reader.bambu".
    const char *id_impl() const noexcept override;
    // Human readable name.
    const char *name_impl() const noexcept override;
    // One-line description.
    const char *description_impl() const noexcept override;
    // Return 100 when a top comment line (before the first command) holds a
    // Bambu-only marker, otherwise 0.
    uint32_t probe_impl(std::string_view prefix) const override;
    // Identity registered for BAMBU_READER_STATE_NAME: a state of any other
    // identity is rejected before the Reader sees an input chunk.
    uint32_t reader_state_id_impl() const override { return m_reader_state_id; }

    uint32_t m_reader_state_id;
};

}}} // namespace slic3r_api::GCodeGeneration::Reader

#endif // slic3r_Plugins_GCode_Reader_BambuGCodeReader_hpp_
