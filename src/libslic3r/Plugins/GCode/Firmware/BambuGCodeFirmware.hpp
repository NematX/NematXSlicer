///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Firmware_BambuGCodeFirmware_hpp_
#define slic3r_Plugins_GCode_Firmware_BambuGCodeFirmware_hpp_
#include "libslic3r/Plugins/Common/SourceLayerIndices.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "Marlin2GCodeFirmware.hpp"

/*
Bambu Lab firmware session
==========================

Bambu Lab printers run a Marlin 2 derived firmware, so this session reuses the
Marlin 2 encoders (movement, temperatures, `M205` jerk line, ...) and only adds
the commands the Bambu slicers write on top of them:

- object cancellation labels (`M624` / `M625`) and the retraction pair that keeps
  every skippable block balanced, see BambuGCodeFirmware.cpp;
- the spaghetti detector switch (`M981`) around the printed part;
- optional platter and exhaust fans (`M106 P2` / `M106 P3`), configured by the
  firmware provider and kept separate from the part-cooling fan. A custom script
  that writes them is followed too (see script_fan_tool() and
  synchronize_reader_private_state());
- pressure advance in the Bambu form (`M900 K.. L1000 M10`);
- power loss recovery (`M1003`) and the second layer scan (`M976`), only when the
  optional `printer_custom_variables` ask for them.

The paired `gcode.reader.bambu` recognizes Bambu object labels and detector
commands without invalidating the modelled movement state.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Firmware {

class BambuGCodeFirmwareSession : public Marlin2GCodeFirmwareSession
{
public:
    using Marlin2GCodeFirmwareSession::Marlin2GCodeFirmwareSession;
    std::string reader_plugin_id() const override { return "gcode.reader.bambu"; }

    // Build the object rank table from the plan, call the base, then open the
    // spaghetti detector after the whole start sequence.
    // Build ranks before the base can request labels. Borrowed plan filaments supply
    // activated exhaust speeds for owned output and the completion cache; base failures
    // propagate. Source ranks schedule second layers and final passes, closing recovery
    // only when every copy that started recovery has finished.
    std::string begin_print(const Print &print, const ConstPrintingPlan &plan) override;
    // Reset the group's pass cursor before the inherited approach.
    std::string begin_group(const ConstPrintingGroup &group) override;
    // Advance the pass cursor, load its source rank and arm coalesced copy actions.
    // Do not use the previous layer's filament; the selected tool group resolves its
    // fan and consumes pending actions. Preserve inherited text and failures.
    std::string begin_layer(const ConstPrintingLayerGroup &layer) override;
    // Close recovery at a copy's last layer only when no started copy remains.
    // Close the inherited layer first and preserve its output and failures.
    std::string end_layer() override;
    // Select the group's tool, then emit a pending platter speed outside object labels.
    std::string begin_tool_group(const ConstPrintingToolGroup &group) override;
    // Close the spaghetti detector at the end of the last group, before end_gcode,
    // after stopping the platter fan unless it is known to be stopped.
    // Earlier groups emit neither fan stop nor detector close. Preserve base output
    // and failures after it closes any object block; update the fan cache on stopping.
    std::string end_group() override;
    // Set the exhaust fan's completion speed after end_gcode.
    std::string end_print() override;

protected:
    // Copy per-tool retraction and fan settings without retaining the Config.
    void setup_firmware(const Config &config) override;
    // Preserve shared part cooling state and remember the selected filament.
    void synchronize_selected_extruder_state(const DefaultExtruder *previous,
                                             DefaultExtruder &selected) override;
    // Flush ordered tool changes before geometry when no object has opened since.
    // While an object block is open nothing is written (closing it here would leave
    // the rest of the object outside its block); the speed waits for the next
    // opening or tool group.
    std::string visit_extrusion_leaf(const ExtrusionEntity &entity) override;
    // Flush the platter speed outside the block that is about to open.
    std::string write_before_object_start() override;
    // The depth every labelled block opens with: retract_length of the tool.
    double object_block_retract_length(const ObjectLabel &object,
                                       extruder_index_t tool_id) const override;
    // In a script, `M106`/`M107` without P and with P1 address the part fan of the
    // selected tool (unknown selection: the neutral reading); P2 and P3 are the
    // platter and exhaust fans, not the fans of the tools 2 and 3.
    std::optional<int32_t> script_fan_tool(std::optional<int32_t> fan_index,
                                           std::optional<extruder_index_t> selected_tool) const override;
    // Give the Reader the platter speed written last, when it is known, so that a
    // script which makes the Reader lose its state is detected afterwards.
    void append_reader_private_state(
        std::vector<raw_gcode_reader_event_item> &items) const override;
    // Adopt the platter speed a custom script left (`M106 P2` / `M107 P2`) as the
    // speed written last, so the next change is compared with the real fan; the
    // speed becomes unknown when the script changed it inside an object block or
    // when the Reader lost it.
    void synchronize_reader_private_state(const GCodeStateTracker &state) override;
    // A tool change is never written inside a block: the printer would skip it with a
    // cancelled object.
    bool closes_object_block_before_tool_change() const override { return true; }
    // Bambu writes no object declaration in the executable G-code.
    std::string encode_object_definitions(const std::vector<ObjectLabel> &objects) const override;
    // `; start printing object, unique label id: <id>` + `M624 <base64 mask>`; nothing
    // for an object without rank.
    std::string encode_object_start(const ObjectLabel &object) const override;
    // `; stop printing object, unique label id: <id>` + `M625`; nothing without rank.
    std::string encode_object_end(const ObjectLabel &object) const override;
    // A labelled object is closed with M625 followed by `G92 E0` in absolute E.
    bool resets_extrusion_axis_after_object_end(const ObjectLabel &object) const override;
    // `M900 K<pa> L1000 M10 ; ...`; the tool is ignored, K has 4 significant digits.
    std::string encode_pressure_advance(extruder_index_t tool_id,
                                        double pressure_advance) const override;

private:
    // Emit a changed pending platter speed after closing any current object block.
    std::string write_platter_fan();
    // On the first tool group of a pass with new second-layer copies: one recovery/scan sequence,
    // outside object blocks, as requested by printer_custom_variables.
    // Close an open block first so cancellation cannot skip commands. M1003 precedes
    // the scan, which uses write_with_retraction() with the selected tool's length:
    // an unretracted tool gets a pair, otherwise at most the complement. No requested
    // action emits nothing; base failures propagate.
    std::string write_second_layer_commands();
    // Rank of an object in the bitmask, or -1 when the object carries no label
    // (synthetic, or more than 64 real objects in the plan).
    int32_t object_rank(uint32_t catalog_id) const;

    // Indexed by plan catalog id: position of that id among the real objects
    // sorted by id, or -1. Empty when labels are disabled (more than 64 objects).
    std::vector<int32_t> m_object_ranks;
    // retract_length of each tool, copied in setup_firmware().
    std::vector<double> m_retract_lengths;
    // Number of plan groups (set in begin_print) and number of end_group() calls
    // seen so far; the last group is the one that closes the spaghetti detector.
    uint32_t m_group_count = 0;
    uint32_t m_ended_group_count = 0;
    bool m_platter_fan = false;
    bool m_support_air_filtration = false;
    std::vector<int> m_platter_fan_speeds;
    std::vector<int> m_disable_fan_first_layers;
    std::vector<bool> m_activate_air_filtration;
    std::vector<int> m_during_print_exhaust_fan_speeds;
    std::vector<int> m_complete_print_exhaust_fan_speeds;
    extruder_index_t m_selected_filament = 0;
    int m_group_layer = -1;
    uint32_t m_source_layer_index = 0;
    bool m_platter_fan_pending = false;
    // From printer_custom_variables (setup_firmware): `scan_first_layer` is 1/true;
    // `enable_power_loss_recovery` gives the `M1003` argument, -1 when it writes nothing.
    bool m_scan_first_layer = false;
    int m_power_loss_recovery = -1;
    // Set by begin_layer() from the precomputed per-copy starts; the first tool group
    // consumes it in write_second_layer_commands(), whether or not a variable asks for
    // commands (then nothing is written).
    bool m_second_layers_pending = false;
    // One pass's minimum merged object/auxiliary source rank, count of newly
    // second-layer copies, and recovery-close decision over still-active copies.
    struct CopyCommands { uint32_t source_index = 0; uint32_t second_layers = 0; bool close_recovery = false; };
    // Per-print commands cached by plan group, then layer, and consumed during traversal.
    std::vector<std::vector<CopyCommands>> m_copy_commands;
    // Speed written last (0..255), or -1 when unknown: initially, since even an
    // initial zero must be sent once, and after a script left it uncertain.
    int m_last_platter_fan_speed = -1;
    int m_completion_exhaust_speed = -1;
};

}}} // namespace slic3r_api::GCodeGeneration::Firmware

#endif // slic3r_Plugins_GCode_Firmware_BambuGCodeFirmware_hpp_
