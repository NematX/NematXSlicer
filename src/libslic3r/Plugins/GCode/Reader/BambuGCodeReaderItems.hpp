///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Plugins_GCode_Reader_BambuGCodeReaderItems_hpp_
#define slic3r_Plugins_GCode_Reader_BambuGCodeReaderItems_hpp_

#include <cstdint>

#include "libslic3r/Api/plugin/c/slic3r_gcode_reader.h"

/*
Bambu fan items
===============

Bambu printers address three fans with `M106 P<n>`: `P1` (or no `P`) is the part
cooling fan of the selected tool, `P2` is the auxiliary ("platter") fan and `P3`
the exhaust fan. The last two are not tool fans, so the Bambu Reader never
publishes them as RAW_GCODE_READER_ITEM_FAN_SPEED, whose target is a tool.

This header is the contract between `gcode.reader.bambu` (it publishes the items)
and `bambu` (its session exports and reads them around a custom
script). Each item has target 0 and holds a percentage in 0..100, like FAN_SPEED.

The items are private to the Bambu schema: a state may carry them only when its
Reader identity is the one registered for BAMBU_READER_STATE_NAME. The Reader and
the firmware register that same name independently in the orchestrator's
GCODE_READER_STATE_ID_TYPE registry; the returned number is process-local and must
never be stored. The Reader rejects an initial state of another identity, and the
session rejects a final state of another identity, so the items cannot reach
another Reader. The Bambu schema has only these two keys and no state class.

Reading an item after a script: a present item is the final speed the Reader knows.
An absent item means that no final value is known: the script set none, or the
Reader lost it (an unknown command discards every private value). The presence or
absence of a command is not deduced from it. The session compares the item with the
value it exported before the script: an equal value has no effect.
*/

namespace slic3r_api { namespace GCodeGeneration { namespace Reader {

// Stable schema name of the Bambu private Reader state.
constexpr const char *BAMBU_READER_STATE_NAME = "slic3r.bambu";

enum BambuGCodeReaderItem : uint32_t {
    // `M106 P2 S..` / `M107 P2`: auxiliary (platter) fan speed in percent.
    BAMBU_ITEM_PLATTER_FAN_SPEED = RAW_GCODE_READER_ITEM_CUSTOM_BEGIN,
    // `M106 P3 S..` / `M107 P3`: exhaust fan speed in percent.
    BAMBU_ITEM_EXHAUST_FAN_SPEED
};

}}} // namespace slic3r_api::GCodeGeneration::Reader

#endif // slic3r_Plugins_GCode_Reader_BambuGCodeReaderItems_hpp_
