///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_GCodePreamble_hpp_
#define slic3r_GCodePreamble_hpp_
#include <string>
#include <vector>
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
    std::string begin(const Print &print,
                       const ConstPrintingPlan &plan,
                      const std::array<std::string, 3> &thumbnail_blocks);
    std::string finish();
    std::string pre_start(const std::string &resolved);
    std::string post_start();
    // Prepare the host context, fill firmware execution values except producer-frozen layer values, run the script, and reconcile machine state.
    std::string process_script(gcode_script_type type, const std::string &script,
        const Config *producer_config, extruder_index_t processing_tool);
private:
    DefaultGCodeFirmwareSession &m_session;
    const Config *m_config = nullptr;
    std::vector<extruder_index_t> m_initial_tools;
    extruder_index_t m_initial_tool = EXTRUDER_ID_UNKNOWN;
    bool m_in_preamble = false;
    bool m_start_written = false;
    std::string m_trailing_thumbnails;
};
}}
#endif
