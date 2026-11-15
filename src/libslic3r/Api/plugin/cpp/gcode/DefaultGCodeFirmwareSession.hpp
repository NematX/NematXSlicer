///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_gcode_DefaultGCodeFirmwareSession_hpp_
#define slic3r_Api_plugin_cpp_gcode_DefaultGCodeFirmwareSession_hpp_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeFirmwareViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeReaderService.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeFormatter.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/GCodeScriptProcessorViews.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/DefaultExtruder.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/Gantry.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/MachineEnvelope.hpp"
#include "libslic3r/Api/plugin/cpp/gcode/Printer.hpp"

/*
Reusable standard PrintingPlan firmware session
================================================

DefaultGCodeFirmwareSession turns ordered PrintingPlan extrusions into a
modern Marlin-style command stream. The session owns all machine state for one
export: position, E coordinates, selected tool, requested process values,
persistent heater targets and the values already sent to the printer.
Before each custom script, the standard firmware fills execution values such as
object index, per-filament usage, current layer and z-hop; producer-frozen
layer values remain authoritative.

The traversal and state transition algorithm is deliberately non-virtual. A
derived firmware changes syntax by overriding one small protected encoder, for
example encode_fan(), without having to duplicate inheritance, extrusion
accounting or command de-duplication.

The class lives in the public C++ plugin API so another firmware provider may
derive from it and replace only the encoders whose syntax differs. It does not
register a plugin by itself. PrintingPlanFileWriter remains responsible for the
output artifact and writes each returned string in plan order.

Object labels identify cancellable model instances. Synthetic catalog entries
have no source object (wipe towers are synthetic) and are printed outside all
labels; their plan IDs remain available for extrusion lookup, not cancellation.
*/

namespace slic3r_api { namespace GCodeGeneration {

// First limit for support deposits swallowed by a negative E remainder: force a step
// at this fraction of E resolution; may be tuned.
inline constexpr double SUPPORT_DEPOSIT_MIN_STEP_FRACTION = 0.5;
// First limit for support deposits with no emitted E word: dwell at their point
// for this many milliseconds; may be tuned.
inline constexpr double SUPPORT_DEPOSIT_PAUSE_MS = 20.0;

// Owns every state needed to serialize one PrintingPlan export. The session
// interprets extrusion trees, updates the generic machine-state objects and
// delegates only command syntax to its protected encode_* methods. Derived
// firmware classes may replace those encoders, but must leave traversal,
// requested/encoded state transitions and extrusion accounting to this class.
class GCodePreamble;

class DefaultGCodeFirmwareSession : public GCodeFirmwareSession
{
public:
    DefaultGCodeFirmwareSession();
    explicit DefaultGCodeFirmwareSession(GCodeScriptProcessorView scripts,
                                         storage_handle *storage);
    ~DefaultGCodeFirmwareSession() override;

    // Copy every setting needed during the export and reset all transient
    // machine state. The borrowed Config view is not retained.
    void setup(const Config &config);

    // Initialize export state and declare only real catalog instances for cancellation.
    std::string begin_print(const Print &print, const ConstPrintingPlan &plan) override;
    // Build the plugin-side Reader client used to interpret custom G-code.
    // Both handles remain host-owned and must outlive this firmware session.
    void set_gcode_reader_context(orchestrator_handle *orchestrator,
                                  const print_handle *print);
    // Set the runtime identity of the state schema exchanged with the selected
    // Reader. Provider wiring calls this before the session is published.
    void set_reader_state_id(orchestrator_runtime_id reader_state_id);
    std::string reader_plugin_id() const override { return "gcode.reader.default"; }
    // Update object counters and provide clearance for later groups without materialized travel.
    std::string begin_group(const ConstPrintingGroup &group) override;
    // Emit layer markers, track the maximum plan Z and refresh extrusion/script heights.
    std::string begin_layer(const ConstPrintingLayerGroup &layer) override;
    // Set script height to the first source extrusion for this tool when one exists.
    std::string begin_tool_group(const ConstPrintingToolGroup &tool_group) override;
    // Set script height from the extrusion's source layer before visiting its tree.
    std::string write_extrusion(const ConstPrintingExtrusion &extrusion) override;
    std::string write_event(const ExtrusionEntity &event_root) override;
    std::string end_tool_group() override;
    std::string end_layer() override;
    std::string end_group() override;
    std::string end_print() override;

protected:
    struct ObjectLabel
    {
        // This is the zero-based PrintingPlan catalog ID. The all-bits-one
        // value is reserved for work outside a physical object instance.
        uint32_t id = uint32_t(-1);
        std::string name;
        c_point projected_outline_center{};
        std::vector<c_point> projected_outline;
        // Synthetic entries have no source object and never receive cancellation labels.
        bool is_synthetic = false;
    };

    // Startup policy hooks. Encoders retain dialect-specific command syntax.
    virtual std::string write_pre_start(const std::string &resolved_script);
    virtual std::string write_post_start();
    // Describes one movement after geometry and extrusion have been converted
    // to machine units. It is a short-lived value passed to write_lines() and
    // encode_move(); it does not own or mark any persistent machine state as encoded
    // state. An absent destination or extrusion means that axis did not change.
    struct PreparedMove
    {
        enum class Kind : uint8_t { None, Travel, Wipe, Extrusion };

        Kind kind = Kind::None;
        std::optional<c_vec3d> destination;
        // Quantized E-axis value ready for the firmware encoder. ExtrusionAxisState
        // retains all exact-distance and rounding state internally.
        std::optional<double> extrusion;
        float radius_mm = 0.f;
        raw_extrusion_arc_orientation arc_orientation = RAW_EXTRUSION_ARC_ORIENTATION_UNKNOWN;
    };

    Gantry &gantry() { return m_gantry; }
    const Gantry &gantry() const { return m_gantry; }
    Printer &printer() { return m_printer; }
    const Printer &printer() const { return m_printer; }
    std::vector<DefaultExtruder> &extruders() { return m_extruders; }
    const std::vector<DefaultExtruder> &extruders() const { return m_extruders; }
    DefaultExtruder &current_extruder();
    const DefaultExtruder &current_extruder() const;
    // Report whether logical filament tools share one physical extruder.
    bool single_extruder_multi_material() const { return m_single_extruder_multi_material; }
    std::optional<extruder_index_t> current_extruder_id() const { return m_current_extruder_idx; }

    // Give dialect encoders the same numeric formatting policy as the base
    // implementation without exposing ownership of the session formatter.
    const GCodeFormatter &gcode_formatter() const;
    static std::string format_number(double value, uint16_t precision = 5);

    // Unsupported operations remain visible in generated output. Returning a
    // valid comment also lets the caller mark the request as handled, instead
    // of retrying the same unavailable command before every movement.
    static std::string encode_unsupported_operation(const char *operation);

    // Execute a host-owned placeholder script under an explicit semantic
    // type. Derived firmwares can reuse this for start_gcode, toolchange_gcode
    // and future scoped events without depending on PlaceholderParser.
    std::string process_script(
        gcode_script_type script_type,
        const std::string &script,
        const Config *producer_config = nullptr,
        extruder_index_t processing_extruder_id = EXTRUDER_ID_UNKNOWN);

    // Every script receives its stored structural Config plus the standard
    // machine context. A derived firmware may fill additional runtime options
    // here without changing the generic script processor ABI.
    virtual void complete_script_context(gcode_script_type script_type,
                                         extruder_index_t processing_extruder_id,
                                         GCodeScriptConfig &config);

    // Resolve a configured script whose setting is empty. The neutral session
    // has no firmware policy and returns an empty string; concrete dialects may
    // supply a safe native pause or filament-change command. A non-empty user
    // setting never reaches this hook.
    virtual std::string resolve_empty_script(gcode_script_type script_type,
                                             const Config *producer_config) const;

    // A dialect may copy additional setup values after the generic machine
    // state has been initialized. The borrowed Config view remains valid only
    // for the duration of this call. Overrides must call their direct parent
    // implementation so intermediate firmware policies can cache their state.
    virtual void setup_firmware(const Config &config);

    // Reader-private state travels through the generic tracker as opaque
    // scalar keys. A dialect appends its snapshot before a custom script and
    // imports the validated final snapshot afterwards. Implementations must
    // preserve PrintingPlan requests and update only their encoded knowledge.
    orchestrator_runtime_id reader_state_id() const { return m_reader_state_id; }
    virtual void append_reader_private_state(
        std::vector<raw_gcode_reader_event_item> &items) const;
    virtual void synchronize_reader_private_state(const GCodeStateTracker &state);
    // Name the tool whose part-cooling fan a script line `M106` / `M107` addresses.
    // `fan_index` is the value of the P parameter when the line has one;
    // `selected_tool` is the tool selected where the script runs (the current tool,
    // else the tool the script was processed for), empty when none is known. The
    // answer is not range checked: the caller ignores a tool the print does not
    // have. An empty answer means the command is not a tool fan. The neutral
    // session reads P as the tool index, and a line without P addresses the
    // selected tool; a dialect whose P is a fan number overrides it.
    virtual std::optional<int32_t> script_fan_tool(std::optional<int32_t> fan_index,
                                                   std::optional<extruder_index_t> selected_tool) const;
    // True while a firmware-labelled object block is open: the printer skips the
    // text written now when that object is cancelled.
    bool object_block_open() const { return m_object_block_open; }
    // Update the protocol-level G10/G11 flag reconstructed by a dialect
    // Reader. This does not invent an E position or retraction distance.
    void synchronize_native_retraction(extruder_index_t tool_id, bool retracted);

    // Convert validated printer limits into this dialect's startup commands.
    // The neutral base emits nothing; concrete firmware sessions choose both
    // the supported fields and their command syntax.
    virtual std::string encode_machine_envelope(const MachineEnvelope &envelope) const;

    // These hooks let a derived firmware observe additional property types
    // without replacing the extrusion-tree visitor. A derived implementation
    // may maintain its own inheritance stack: push or override state on entry,
    // consume it before a leaf is serialized, then restore it on exit. The
    // default hooks emit no text and keep no additional state.
    virtual std::string enter_extrusion_node(const ExtrusionEntity &entity);
    virtual std::string visit_extrusion_leaf(const ExtrusionEntity &entity);
    virtual std::string leave_extrusion_node(const ExtrusionEntity &entity);

    // Encode and acknowledge the acceleration required by this movement. The
    // standard implementation preserves independent print and travel
    // histories; specialized sessions may override their relationship without
    // changing the extrusion traversal performed by write_lines().
    virtual std::string write_acceleration(PreparedMove::Kind kind);

    // Apply additional dialect sharing after the base transfers SEMM runtime state.
    virtual void synchronize_selected_extruder_state(const DefaultExtruder *previous,
                                                      DefaultExtruder &selected);

    // The methods below encode one already-decided state transition. They must
    // not mutate session state; write_lines() validates state only after an
    // encoder returns successfully.
    virtual std::string encode_tool_change(extruder_index_t tool_id) const;
    // Encode a coordinate-only reset of the active extrusion axis to zero.
    virtual std::string encode_extrusion_axis_reset() const;
    // Encode a heater target, omitting the logical tool address for a shared SEMM heater.
    virtual std::string encode_tool_temperature(extruder_index_t tool_id,
                                                int16_t temperature,
                                                bool wait) const;
    virtual std::string encode_bed_temperature(int16_t temperature, bool wait) const;
    virtual std::string encode_chamber_temperature(int16_t temperature, bool wait) const;
    virtual std::string encode_fan(extruder_index_t tool_id, double speed_percent) const;
    virtual std::string encode_pressure_advance(extruder_index_t tool_id,
                                                double pressure_advance) const;
    virtual std::string encode_acceleration(uint32_t acceleration, bool travel) const;
    virtual std::string encode_move(const PreparedMove &move, bool include_speed) const;
    virtual std::string encode_custom_gcode(c_extrusion_custom_gcode_kind kind,
                                            const std::string &text) const;
    // Encode a speed multiplier without backup in dialects lacking firmware save support.
    virtual std::string encode_save_speed_ratio(double ratio) const;
    // Encode the 100% speed fallback in dialects lacking firmware restore support.
    virtual std::string encode_restore_speed_ratio() const;
    virtual std::string encode_flush_planner() const;
    virtual std::string encode_pause(double milliseconds) const;
    virtual std::string encode_extruder_current(extruder_index_t tool_id, double current) const;
    /* Encode one native firmware retract or unretract request, normally G10/G11. */
    virtual std::string encode_firmware_retraction(bool retract) const;
    // Encode dialect-specific object declarations and boundaries. The common
    // session adds Octoprint comments according to gcode_label_objects, while
    // derived firmware classes provide only their native protocol.
    virtual std::string encode_object_definitions(const std::vector<ObjectLabel> &objects) const;
    virtual std::string encode_object_start(const ObjectLabel &object) const;
    virtual std::string encode_object_end(const ObjectLabel &object) const;
    // Ask whether the firmware end marker written by encode_object_end() must
    // be followed by an absolute-E zero. When true and E is absolute, the session
    // appends the dialect's axis reset and zeroes its logical E before preparing
    // any later extrusion, so a firmware that skips a cancelled object's moves
    // cannot see the skipped E distance. Retraction state is preserved. Relative
    // E never needs it. The neutral session answers false.
    virtual bool resets_extrusion_axis_after_object_end(const ObjectLabel &object) const;
    // Retraction depth, in mm, that the labelled block of `object` must open with
    // for the tool `tool_id`; 0 (the neutral answer) leaves the block alone. When it
    // is positive and encode_object_start() writes a label, the session:
    //   - retracts the complement up to this depth just BEFORE the start label, only
    //     if the tool is less retracted (a retraction already at least this deep is
    //     kept), by explicit E or by the native retract command;
    //   - then, inside the block, after the approach and before the first deposited
    //     geometry, repays any remaining retraction with an explicit unretraction
    //     (E move or native unretract), so the first extrusion is not shortened.
    //   - before the end label (inside the block, before the axis reset), retracts the
    //     complement up to the depth the block was entered with (its debt after the
    //     entry retraction, so at least this depth); a tool change closes the block
    //     first (see select_extruder()), so the tool is still the block's tool.
    // Native retraction already active is kept as is: no logical complement.
    // Invariant for a dialect that answers a depth: every labelled block ends with the
    // debt it started with, so the printer can skip it without changing the filament
    // state. A block that ends deeper than its entry depth (the plan's tool-change
    // retraction inside the block before a tool change) is not normalized.
    virtual double object_block_retract_length(const ObjectLabel &object,
                                               extruder_index_t tool_id) const;
    // Ask whether a firmware-labelled block must be closed (exit work, end label, axis
    // reset) before the session selects another tool, whatever its retraction depth,
    // so that a skipped object never skips the tool change. The neutral session
    // answers false and leaves tool changes where the plan puts them.
    virtual bool closes_object_block_before_tool_change() const;

    // Emit dialect text outside a cancellable block, immediately before its opening.
    virtual std::string write_before_object_start();
    // Close/open labels, treating synthetic IDs and UINT32_MAX as outside objects.
    std::string transition_object(uint32_t printing_catalog_id);
    // Wrap dialect text in a retraction/unretraction pair of the current tool, so that a
    // pause of the print (a scan, a probe...) happens with the filament pulled back.
    //   - tool not retracted: retract `retract_length` mm (explicit E, or G10 with
    //     firmware retraction), then `gcode`, then repay that temporary retraction only
    //     (E move or G11); restart extras scheduled before stay scheduled;
    //   - tool already retracted, even partially: only the complement up to
    //     `retract_length` (nothing if it is at least that deep), then `gcode`; the
    //     existing debt is left for the normal flow of the following extrusion;
    //   - firmware retraction already active: nothing is added (the firmware's depth is
    //     kept, as for labelled blocks), then `gcode`;
    //   - `retract_length` of 0 (or less): `gcode` alone.
    // Contract: `retract_length` is finite and >= 0; `gcode` must not change the tool, the
    // E position or the E mode (it is only appended, never interpreted); the caller closes
    // any open object block first (transition_object()) when the text must not be skipped
    // with an object. Requires a selected tool.
    std::string write_with_retraction(const std::string &gcode, double retract_length);

private:
    std::unique_ptr<GCodeReaderService> m_gcode_reader_service;
    friend class GCodePreamble;
    std::unique_ptr<GCodePreamble> m_preamble;
    struct RequestedState;
    class ExtrusionWriterVisitor;
    class GCodeStateInterpreter;

    // Select a tool, transfer SEMM runtime, settle filament usage and reset absolute E after emitted selections.
    std::string select_extruder(extruder_index_t tool_id, bool emit_command = true);
    // Find the first geometric leaf of a borrowed group. The first non-empty group
    // establishes the sequence; later groups without explicit Travel geometry rise
    // vertically to bounded running-height clearance, then move XY at that height.
    // The ordinary first-path approach descends at the destination. Missing machine
    // position/tool emits nothing; encoding failures propagate and no tree is retained.
    // Arrival XY uses the first geometry's tool frame, not the outgoing tool's frame.
    // Retain clearance for a start-point correction after selection even when explicit
    // travel suppresses this fallback movement.
    std::string move_to_group_start(const ConstPrintingGroup &group);
    // Visit an extrusion tree while tracking its object for opt-in tool-change reopening.
    std::string write_extrusion_tree(const ExtrusionEntity &root,
                                     uint32_t object_id = uint32_t(-1));
    // Emit changed role/dimension tags, permitting zero dimensions for fixed-E paths.
    std::string write_printable_markers(const EPropertyAttributes &attributes, bool fixed_e = false);
    std::string write_seam_marker(const ExtrusionEntity &loop) const;
    std::string transition_wipe_marker(bool wipe);
    // Emit required state commands and a prepared move, restoring a zero E origin when unknown.
    std::string write_lines(const PreparedMove &move);
    // Emit planar segments with flow-derived E, a fixed filament delta, or semantic retraction.
    std::string write_leaf_geometry(const ExtrusionEntity &leaf, const RequestedState &state);
    // Convert a scaled point and Z offset to machine coordinates and return approach
    // commands only when the quantized position differs; owned gantry state advances,
    // inherited speed is restored, and encoding failures propagate. The first approach
    // raises vertically to lift_min before moving XY, then descends at the destination.
    // An unknown start position uses a Z-only absolute-mm move. Floor and ceiling use
    // the requested path's gantry and tool corrections from print Z to machine Z.
    // Consume pending group-entry clearance for XY corrections after tool selection,
    // including explicit travel starts; matching XY leaves that travel responsible for lift.
    std::string approach_leaf_point(c_point point, coord_t z_offset);
    // Emit a stationary volume, forcing swallowed support steps or dwelling for sub-step deposits.
    std::string write_deposit(const ExtrusionEntity &leaf, const RequestedState &state);
    // Retract the active tool of a labelled block down to `depth` mm. Does nothing
    // when native retraction is already active: the firmware keeps its own depth and
    // no logical complement is recorded.
    std::string write_block_retraction(double depth);
    // Retract the active tool down to `target` mm by explicit E, or by the native
    // command in firmware retraction; writes nothing when it is already that deep.
    std::string write_retraction(double target);
    // Repay the whole retraction of the active tool (explicit E or native unretract);
    // writes nothing when it is not retracted.
    std::string write_full_unretraction();
    // Emit static retraction or full/limited unretraction for the active tool.
    std::string write_extrusion_axis_event(const EPropertyExtrusionAxis &axis,
                                           const RequestedState &state);
    // Apply one entry heater event immediately; its target persists and is not inherited or restored.
    std::string write_heater_event(const EPropertyHeater &heater);
    // Update the semantic E state and emit at most one native retract command
    // while the selected tool remains retracted.
    std::string write_native_retract(double target);
    // Restore the semantic E state and emit the matching native unretract.
    std::string write_native_unretract();
    // Emit an entry command, rejecting DEPOSIT because it requires a one-point leaf.
    std::string write_special_command(const EPropertySpecialCommand &command,
                                      const RequestedState &state);
    // Emit a script, reconcile machine state and establish its post-toolchange E origin.
    // The entity only borrows the script
    // data; this session never takes ownership of it.
    std::string write_custom_gcode(const ExtrusionEntity &entity,
                                   const EPropertyCustomGcode &custom_gcode);
    // Look up session-owned object metadata, rejecting unknown catalog IDs.
    const ObjectLabel &object_label(uint32_t printing_catalog_id) const;

    // Reconcile custom G-code after it was interpreted. Replace each cached
    // value with the reader's final known value; the retraction state stays
    // whatever the explicit retraction operations and declarations set,
    // because the reader cannot prove it. Unknown reader values deliberately
    // invalidate the corresponding cache so later generated commands establish
    // a safe, explicit state. This does not emit G-code, settles filament usage
    // after a reported tool change, and is a no-op when no reader service is available.
    void synchronize_after_custom_gcode(const std::string &text);
    // Close the current per-tool filament interval and start measuring the selected tool's axis; call immediately after every m_current_extruder_idx change outside setup so m_usage_tool follows, and SEMM state copying does not alter the departing tool's axis.
    void settle_filament_usage();
    // Return per-tool filament lengths, including the currently open interval, without changing session counters.
    std::vector<double> filament_used_mm() const;
    void apply_requested_state(const RequestedState &state);
    std::string write_requested_fan();
    c_vec3d machine_position(c_point point, coord_t z_offset) const;
    double segment_length_mm(const c_extrusion_segment &segment) const;
    DefaultExtruder &extruder(extruder_index_t tool_id);
    const DefaultExtruder &extruder(extruder_index_t tool_id) const;

    Gantry m_gantry;
    Printer m_printer;
    std::vector<DefaultExtruder> m_extruders;
    std::optional<extruder_index_t> m_current_extruder_idx;
    std::unique_ptr<GCodeFormatter> m_formatter;
    coord_t m_layer_print_z = 0;
    std::optional<std::string> m_marker_role;
    std::optional<float> m_marker_width;
    std::optional<float> m_marker_height;
    std::vector<double> m_last_layer_used_filament;
    // Filament length used by each tool over its closed selection periods. In SEMM the
    // logical tools share one E axis, so an axis total mixes filaments; the open period
    // of m_usage_tool is measured on its axis from m_usage_start_mm. E moves from a
    // script with no current tool, or addressed by T to a tool that is not selected,
    // are assigned to no tool, as in the legacy generator.
    std::vector<double> m_filament_used_mm;
    std::optional<extruder_index_t> m_usage_tool;
    double m_usage_start_mm = 0.0;
    // Zero-based index of the sequential object being printed, like the legacy current_object_idx.
    int32_t m_current_object_idx = 0;
    // Current plan layer for scripts: begin_layer() count minus one (-1 before the first layer) and its height; m_layer_print_z already holds its print Z.
    int32_t m_layer_num = -1;
    coord_t m_layer_height = 0;
    std::optional<extruder_index_t> m_script_processing_tool;
    bool m_xyz_relative_mode = false;
    bool m_units_in_mm = true;
    bool m_e_relative_mode = false;
    // Cleared when external G-code invalidates E. The next generated extrusion
    // establishes a fresh E origin instead of continuing a stale coordinate.
    bool m_e_position_known = true;
    bool m_seen_object_group = false;
    bool m_seen_print_group = false;
    // Per-print clearance state in print coordinates before machine correction:
    // running maximum Z is scaled; entry clearance, tool-indexed retract lifts,
    // the absolute lift floor and printer ceiling are millimeters. The optional
    // entry clearance lasts until the next group; the first-approach flag lasts
    // until the first geometry approach. begin_print() resets these values.
    coord_t m_running_max_print_z = 0;
    std::optional<double> m_group_entry_clearance;
    std::vector<double> m_retract_lifts;
    double m_lift_min = 0.0;
    double m_max_print_height = 0.0;
    bool m_first_approach = true;
    bool m_use_firmware_retraction = false;
    // SEMM means single-extruder multi-material: tool IDs select filaments,
    // while heater, fan, pressure advance and extrusion state share hardware.
    bool m_single_extruder_multi_material = false;
    // G10/G11 expose one binary protocol state per tool even though the
    // semantic retraction target may advance through several plan events.
    std::vector<uint8_t> m_native_retraction_active;
    bool m_wipe_tag_open = false;
    enum class ObjectLabelStyle { Disabled, Octoprint, Firmware, Both };
    ObjectLabelStyle m_object_label_style = ObjectLabelStyle::Disabled;
    std::vector<ObjectLabel> m_object_labels;
    // Plan-local object ID currently selected; UINT32_MAX means outside any
    // physical object instance. The generated marker uses this same ID and
    // writes -1 for the outside sentinel.
    uint32_t m_current_printing_catalog_id = uint32_t(-1);
    // Object of the tree being visited, independent of temporarily closed labels.
    uint32_t m_extrusion_tree_object_id = uint32_t(-1);
    // Only an opt-in tool-change closure requests reopening before the next geometry.
    bool m_object_block_reopen_pending = false;
    // Set when the open block was started with a retraction depth (see
    // object_block_retract_length): its first deposited geometry must be preceded
    // by an unretraction if a debt remains. Cleared when that geometry is reached
    // or the block closes.
    bool m_object_block_unretract_pending = false;
    // Entry depth (mm of debt right after the entry retraction) of the open block
    // when it was started with a retraction depth; the exit retraction goes back to
    // it. Unset when no block with a depth is open.
    std::optional<double> m_object_block_depth;
    // True while a block that got a firmware start label is open.
    bool m_object_block_open = false;
    bool m_is_setup = false;
    orchestrator_runtime_id m_reader_state_id = GCODE_READER_STATE_ID_PORTABLE;
    GCodeScriptProcessorView m_scripts;
    // Reused for one script at a time. The processor clones every option during
    // prepare(), so this storage may be cleared before the following script.
    // works only because this step isn't run in parallel with the same instance.
    std::optional<StoredConfig> m_script_config;
};

}} // namespace slic3r_api::GCodeGeneration

#endif // slic3r_Api_plugin_cpp_gcode_DefaultGCodeFirmwareSession_hpp_
