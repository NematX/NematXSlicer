///|/ Copyright (c) NematXSlicer 2026 Durand Remi @supermerill
///|/
///|/ NematXSlicer is released under the terms of the AGPLv3 or higher

#ifndef nematx_firmware_NematXGCodeReader_hpp_
#define nematx_firmware_NematXGCodeReader_hpp_

#include <optional>
#include <string>
#include <string_view>

#include "libslic3r/Api/plugin/cpp/gcode/BuiltinGCodeReaderBase.hpp"

namespace nematx_firmware {

// Reads NematX G-code for the preview, the statistics, and the firmware
// session, which replays custom G-code scripts through it to keep its machine
// state exact.
//
// The SDK reader base does the work common to all dialects (lines, comments,
// preview markers, standard commands). This class only:
// - removes the "N<k> " line numbers before each line is read;
// - translates NematX lines (axis letters, '=' values, bare waits,
//   SET_POSITION, inline G90/G91) into standard actions (translate_line()).
class NematXGCodeReader final : public slic3r_api::GCodeGeneration::Reader::BuiltinGCodeReaderBase
{
public:
    explicit NematXGCodeReader(orchestrator_handle *orchestrator);

protected:
    void read_impl(const plugin_run_context &run_context, const run_ctx_gcode_reader &context,
                   slic3r_api::GCodeStateTracker &state) const override;
    bool consume_text_command(const plugin_run_context &run_context, const run_ctx_gcode_reader &context,
                              slic3r_api::GCodeStateTracker &state,
                              const slic3r_api::GCodeGeneration::GCodeLineParser::GCodeLine &line) const override;
    bool is_neutral_command(char letter, uint32_t number) const override;
    std::optional<raw_gcode_reader_marker_type> marker_type(char letter, uint32_t number) const override;

private:
    const char *id_impl() const noexcept override;
    const char *name_impl() const noexcept override;
    const char *description_impl() const noexcept override;
    uint32_t probe_impl(std::string_view prefix) const override;

    // Dispatch one standard command produced by the translation, as the base
    // would for a line of the file.
    void dispatch_standard(const plugin_run_context &run_context, const run_ctx_gcode_reader &context,
                           slic3r_api::GCodeStateTracker &state, const std::string &text) const;
};

} // namespace nematx_firmware

#endif // nematx_firmware_NematXGCodeReader_hpp_
