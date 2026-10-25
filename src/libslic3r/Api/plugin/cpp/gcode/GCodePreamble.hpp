///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_GCodePreamble_hpp_
#define slic3r_GCodePreamble_hpp_
#include <string>
#include "GCodeScriptProcessorViews.hpp"
#include "GCodeThumbnailViews.hpp"
#include "../PrintingPlanViews.hpp"
namespace slic3r_api { namespace GCodeGeneration {
class DefaultGCodeFirmwareSession;
// Shared startup and script processing. The session owns this helper; it borrows
// its machine states. Print/config/plan views are only used inside begin().
// A session is used serially. No geometry is created, sorted or retained here.
class GCodePreamble {
public:
    explicit GCodePreamble(DefaultGCodeFirmwareSession &session) : m_session(session) {}
    // Compose the borrowed plan's thumbnails and start events, rejecting unresolved tools and clearing borrowed state afterward.
    std::string begin(const Print &print,
                       const ConstPrintingPlan &plan,
                      const std::array<std::string, 3> &thumbnail_blocks);
    // Transfer deferred trailing thumbnail text to the caller; a second call returns an empty string.
    std::string finish();
    // Emit only missing machine modes before the resolved start script and update the firmware session.
    std::string pre_start(const std::string &resolved);
    // Restore startup modes, select the initial tool, and emit that tool's first-layer fan state.
    std::string post_start();
    std::string process_script(gcode_script_type type, const std::string &script,
        const Config *producer_config, extruder_index_t processing_tool);
private:
    DefaultGCodeFirmwareSession &m_session;
    const Config *m_config = nullptr;
    extruder_index_t m_initial_tool = EXTRUDER_ID_UNKNOWN;
    bool m_in_preamble = false;
    bool m_start_written = false;
    std::string m_trailing_thumbnails;
};
}}
#endif
