///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_GCodePreamble_hpp_
#define slic3r_GCodePreamble_hpp_
#include <string>
#include "GCodeScriptProcessorViews.hpp"
#include "../PrintingPlanViews.hpp"

/*
GCodePreamble handles startup and configured script text for a firmware session.
The session supplies its machine states; this helper reads the Print, settings
and plan during begin(), emits startup commands and existing event trees, and
uses the shared script processor for template expansion. It does not create or
retain printable geometry. Keep the session and its borrowed states alive for
the helper's whole use, and use one session serially.
*/
namespace slic3r_api { namespace GCodeGeneration {
class DefaultGCodeFirmwareSession;
// Shared startup and script processing. The session owns this helper; it borrows
// its machine states. Print/config/plan views are only used inside begin().
// A session is used serially. No geometry is created, sorted or retained here.
class GCodePreamble {
public:
    explicit GCodePreamble(DefaultGCodeFirmwareSession &session) : m_session(session) {}
    // Compose the borrowed plan's startup modes and start events, rejecting unresolved tools and clearing borrowed state afterward.
    std::string begin(const Print &print, const ConstPrintingPlan &plan);
    // Emit only missing machine modes before the resolved start script and update the firmware session.
    std::string pre_start(const std::string &resolved);
    // Restore startup modes, select the initial tool, and emit that tool's first-layer fan state.
    std::string post_start();
    // Prepare the host context, fill firmware execution values except producer-frozen layer values, run the script, and reconcile machine state.
    std::string process_script(gcode_script_type type, const std::string &script,
        const Config *producer_config, extruder_index_t processing_tool);
private:
    DefaultGCodeFirmwareSession &m_session;
    const Config *m_config = nullptr;
    extruder_index_t m_initial_tool = EXTRUDER_ID_UNKNOWN;
    bool m_in_preamble = false;
    bool m_start_written = false;
};
}}
#endif
