///|/ Released under the GNU Affero General Public License, version 3 or later.

// Assembles the pressure components for one ordered export, without a Reader
// service or firmware hook. Construct Flow with copied, caller-resolved settings,
// then consume SourceFragment values from consecutive read_fragment/index_fragment
// calls. Hints are empty or indexed by physical row. Begin opens the layer before
// its text, End closes after it, and EndPrint closes before its outside-layer text.
// Concatenate returned bytes in order; issues own their data. An exception returns
// no bytes for that call and permanently disables this instance. Earlier calls
// cannot be withdrawn: the export owner must withhold its final file on failure.
#pragma once

#include "PressureClassification.hpp"
#include "PressureControls.hpp"
#include "PressureExtrusion.hpp"
#include "PressureGeometry.hpp"
#include "PressureNominal.hpp"
#include "PressurePlanner.hpp"
#include "PressureSource.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace nematx_pressure {

// Configured K is in seconds, before overrides.
// annotate enables NX_PRESSURE phase annotations.
struct FlowTool { double pressure_factor; bool annotate; };
// Settings are resolved by the caller. AxisLimits uses mm/s and mm/s²; initial
// acceleration in mm/s² includes the legacy fallback and positive extrusion cap.
struct FlowConfig
{
    std::vector<FlowTool> tools;
    // Reader extrusion letters indexed by the same logical tool as tools.
    std::vector<char> axis_letters;
    // Decimal output precision, independently for XYZ and filament E in mm.
    int xyz_digits;
    int e_digits;
    AxisLimits limits;
    // Resolved use_volumetric_e; active volumetric printing is unsupported.
    bool volumetric;
    double initial_acceleration;
};
// Begin applies before the text, End after it, EndPrint before outside-layer text.
struct FlowBoundary { bool begin_layer = false; bool end_layer = false; bool end_print = false; };
// Nonfatal observations retained for the export owner's diagnostic channel.
enum class FlowIssueKind { InvalidAcceleration, ReaderInvalidation, ReaderDiagnostic, HorizonInsufficient, GeometryMerged };
// Ordinals count all physical source rows in this Flow, starting at zero.
struct FlowIssue
{
    FlowIssueKind kind;
    std::size_t source_ordinal;
    std::size_t line_index;
    std::optional<std::uint32_t> tool;
    std::vector<raw_gcode_reader_event_item> reader_items;
};
// Owns the finalized text prefix and diagnostics produced by one successful call.
struct FlowOutput { std::string bytes; std::vector<FlowIssue> issues; };

class Flow
{
public:
    // Copies resolved settings and creates independent persistent per-tool E states.
    // Requires nonempty equally sized tool/axis vectors, digits 0..9, finite
    // nonnegative K and finite positive initial acceleration; otherwise throws invalid_argument.
    explicit Flow(const FlowConfig &config);
    // Consumes owned original input; returns only finalized text and owned issues.
    // Any failure propagates and makes subsequent calls throw logic_error.
    // A successful EndPrint also closes the instance; later calls throw logic_error.
    FlowOutput consume(SourceFragment source, const std::vector<MoveHint> &hints,
                       FlowBoundary boundary);

private:
    // A part names original analytical phases, never phases of another anchor.
    struct Part { std::size_t ordinal, first_phase, phase_count; };
    struct Print
    {
        NominalMotion motion;
        SourceGeometry source_geometry;
        double k, feed;
        bool annotate, rebase;
        std::optional<PlannedMove> planned;
        GeometryResult geometry;
        std::array<std::vector<Part>, 3> parts;
        bool transferred = false;
    };
    struct Row
    {
        std::shared_ptr<const SourceFragment> source;
        std::size_t local, ordinal;
        bool transfer_transparent;
        std::optional<Print> print;
    };
    struct ToolState
    {
        Extrusion extrusion;
        // Last scheduled nominal endpoint: independent cumulative E in relative
        // mode, external nominal coordinate in absolute mode; both are in mm.
        double sum = 0.;
        // Previous scheduled nominal end in either mode, used to detect absolute discontinuities.
        std::optional<double> nominal_end;
        // Rebase the next print at zero charge after an effective mode change.
        bool reset_frame = false;
    };
    // Projects the whole publication before resolving transfers or advancing E.
    void publication(Publication published, bool final, FlowOutput &output);
    // Resolves collapsed rows forward first, then backward without crossing effects.
    void resolve(bool final);
    // Emits the safe prefix, retaining the last representable source until closure.
    void emit(bool final, FlowOutput &output);
    // Adds real text with LF separation only when another real text follows.
    void append(std::string text, FlowOutput &output);
    // Tests settings and nominal/quantized endpoint continuity for material transfer.
    // A rebase at first's start is preserved; last.rebase would place a new frame
    // inside the merged group and is refused. Forward transfer carries first's rebase.
    bool compatible(const Print &first, const Print &last) const;
    // Closes the planner, resolves its entire remainder, then releases all rows.
    void finish(FlowOutput &output);
    // Tracks real net mode changes, excluding the residual modes of inline prints.
    void modes(const SourceFragment &source, std::size_t row,
               const std::optional<NominalMotion> &motion, slic3r_api::GCodeStateTracker &state);

    const FlowConfig m_config;
    NominalState m_nominal;
    ClassificationState m_classification;
    ControlState m_controls;
    Planner m_planner;
    std::vector<ToolState> m_tools;
    std::deque<Row> m_rows;
    // Only the three mode flags are used here; CycleSnapshot's tool is not consulted.
    std::optional<CycleSnapshot> m_modes;
    std::size_t m_next_ordinal = 0;
    bool m_in_layer = false, m_failed = false, m_ended = false;
    bool m_unterminated = false;
};
} // namespace nematx_pressure
