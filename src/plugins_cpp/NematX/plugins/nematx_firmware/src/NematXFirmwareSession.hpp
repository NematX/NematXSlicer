///|/ Copyright (c) NematXSlicer 2026 Durand Remi @supermerill
///|/
///|/ NematXSlicer is released under the terms of the AGPLv3 or higher

#ifndef nematx_firmware_NematXFirmwareSession_hpp_
#define nematx_firmware_NematXFirmwareSession_hpp_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "libslic3r/Api/plugin/cpp/gcode/DefaultGCodeFirmwareSession.hpp"

#include "NematXSyntax.hpp"

namespace nematx_firmware {

// Plugin ids, shared by the firmware provider, the reader and the session.
constexpr char k_firmware_plugin_id[] = "nematx";
constexpr char k_reader_plugin_id[] = "gcode.reader.nematx";

// Settings defined by this package.
constexpr char k_extruder_axis_key[] = "extruder_axis";
constexpr char k_disable_unused_fan_key[] = "disable_fan_from_unused_extruder";
constexpr char k_line_number_key[] = "gcode_line_number";
constexpr char k_no_comment_key[] = "gcode_no_comment";

// One export in the NematX dialect.
//
// The base session (public SDK) walks the PrintingPlan, keeps the machine state
// and decides which commands are needed. This class writes those commands and,
// when pressure or phase splitting is enabled, reads the ordered text with a
// separate Reader and transforms closed layer cycles before output filtering.
// The nominal base state is not changed by that transformation.
// It also filters the whole output (line numbers, comment removal),
// describes the objects for the NematX controller after the file header, and
// appends the NematX metadata at the end of the file.
//
// Extrusion-axis resets keep the base policy for now (zero at the points chosen
// by the base session); a firmware-decided policy is a later change.
class NematXGCodeFirmwareSession final : public slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession
{
public:
    using Base = slic3r_api::GCodeGeneration::DefaultGCodeFirmwareSession;

    NematXGCodeFirmwareSession(slic3r_api::GCodeGeneration::GCodeScriptProcessorView scripts, storage_handle *storage);

    // Destroys the owned runtime; the borrowed host and Print must outlive this session.
    ~NematXGCodeFirmwareSession() override;
    // Borrows host objects that must outlive this session; retains no run context.
    void set_pressure_reader_context(orchestrator_handle *orchestrator, const print_handle *print);

    std::string reader_plugin_id() const override { return k_reader_plugin_id; }

    // Every output passes through filter(): see OutputFilter in NematXSyntax.hpp.
    std::string begin_print(const slic3r_api::Print &print, const slic3r_api::ConstPrintingPlan &plan) override;
    std::string begin_group(const slic3r_api::ConstPrintingGroup &group) override;
    // Opens the pressure layer before its generated header, without resetting modes.
    std::string begin_layer(const slic3r_api::ConstPrintingLayerGroup &layer) override;
    std::string begin_tool_group(const slic3r_api::ConstPrintingToolGroup &tool_group) override;
    std::string write_extrusion(const slic3r_api::ConstPrintingExtrusion &extrusion) override;
    std::string write_event(const slic3r_api::ExtrusionEntity &event_root) override;
    std::string end_tool_group() override;
    // Closes the layer after its generated tail and returns all retained phases.
    std::string end_layer() override;
    std::string end_group() override;
    // Closes pressure before outside-layer text, flushes the filter and adds metadata.
    std::string end_print() override;

protected:
    // Resets export state and copies validated pressure settings after nominal setup.
    void setup_firmware(const slic3r_api::Config &config) override;
    std::string write_pre_start(const std::string &resolved_script) override;
    std::string write_post_start() override;
    void synchronize_selected_extruder_state(const slic3r_api::GCodeGeneration::DefaultExtruder *previous,
                                             slic3r_api::GCodeGeneration::DefaultExtruder &selected) override;

    std::string encode_tool_change(extruder_index_t tool_id) const override;
    std::string encode_extrusion_axis_reset() const override;
    std::string encode_tool_temperature(extruder_index_t tool_id, int16_t temperature,
                                        bool wait) const override;
    std::string encode_bed_temperature(int16_t temperature, bool wait) const override;
    std::string encode_chamber_temperature(int16_t temperature, bool wait) const override;
    std::string encode_fan(extruder_index_t tool_id, double speed_percent) const override;
    std::string encode_pressure_advance(extruder_index_t tool_id,
                                        double pressure_advance) const override;
    std::string encode_acceleration(uint32_t acceleration, bool travel) const override;
    // Encodes nominal G1 or G2/G3 with I/J center offsets and a private kind hint
    // only when pressure is enabled. Relative E uses inline modes, for example
    // "G1 G90 X10 Y5 G91 A0.3"; pending commands precede the hint and move.
    std::string encode_move(const PreparedMove &move, bool include_speed) const override;
    std::string encode_save_speed_ratio(double ratio) const override;
    std::string encode_restore_speed_ratio() const override;
    std::string encode_flush_planner() const override;
    std::string encode_pause(double milliseconds) const override;
    std::string encode_extruder_current(extruder_index_t tool_id, double current) const override;
    std::string encode_firmware_retraction(bool retract) const override;
    std::string encode_machine_envelope(const slic3r_api::GCodeGeneration::MachineEnvelope &envelope) const override;

private:
    // Axis letter of a tool, or of the current tool when none is given.
    char axis_letter(extruder_index_t tool_id) const;
    char current_axis_letter() const;
    // A shared-heater/fan printer (single extruder multi material) only has the
    // first tool's commands.
    size_t physical_tool(extruder_index_t tool_id) const;
    // Text queued between two encoders (see synchronize_selected_extruder_state).
    std::string take_pending() const;
    enum class Boundary { Fragment, BeginLayer, EndLayer, EndPrint };
    // Reads and transforms cleaned production once, then applies the output filter.
    std::string filter(std::string text, Boundary boundary);
    // The base may call write_event inside begin_print. Leave nested production
    // untouched until its outer owner can read, transform and filter all the text.
    template<class Write> std::string filtered(Write write, Boundary boundary = Boundary::Fragment);

    struct PressureRuntime;
    // Owns the optional Reader and pressure Flow for this export.
    std::unique_ptr<PressureRuntime> m_pressure;
    // Borrowed host objects remain live until session destruction.
    orchestrator_handle *m_pressure_orchestrator = nullptr;
    const print_handle *m_pressure_print = nullptr;
    // Last nonempty pre-filter output needs a separator before metadata when active.
    bool m_output_unterminated = false;

    std::vector<char> m_axis_letters;
    bool m_disable_unused_fan = false;
    bool m_fan_percentage = false;
    bool m_relative_e = false;
    // Object and plater JSON lines, written when objects are labelled.
    bool m_object_metadata = false;
    OutputFilter m_filter;
    int m_filter_depth = 0;
    // Commands created by a state hook, which cannot return text, and written
    // before the next command. mutable: encoders are const but must emit it.
    mutable std::string m_pending;
};

} // namespace nematx_firmware

#endif // nematx_firmware_NematXFirmwareSession_hpp_
