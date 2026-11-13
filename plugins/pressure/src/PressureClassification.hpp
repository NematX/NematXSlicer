///|/ Released under the GNU Affero General Public License, version 3 or later.

// Classifies owned SourceFragment lines using the matching nominal double read.
// The caller resolves configuration and supplies a hint for every generated move;
// scripts may have None. An empty hint vector means all lines have None.
// Pass final by value to the next fragment of the same export, for example:
//   ClassificationFragment next = classify_fragment(source, nominal, hints,
//                                                   context, prior);
// Source and nominal must describe exactly the same read, export and predecessor.
// Decisions contain only owned scalars and indices, never text or borrowed views.
// This component neither closes pressure mathematically nor rewrites any line.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

namespace nematx_pressure {
struct SourceFragment;
struct NominalFragment;

// Producer movement kind for one physical row; None leaves script intent to
// source/Reader proofs. A hint does not prove an active tool or any E mode.
enum class MoveHint { None, Travel, Wipe, Extrusion };

// The caller resolves the active layer zone and configuration before this call.
struct ClassificationContext
{
    // True only in the layer content where eligible printing is classified.
    bool in_layer;
    // Resolved use_volumetric_e, not a default inferred from Reader state.
    bool config_volumetric;
    // One eligibility boolean per tool: positive configured pressure factor or
    // enabled phase splitting, independent of pressure overrides.
    std::vector<bool> enabled_tools;
};

// Proven active tool and pre-motion modes of the last accepted print, not the row's final state.
struct CycleSnapshot
{
    std::uint32_t tool;
    bool xyz_absolute, e_absolute, units_mm;
};

// Carries the last accepted print's cycle and controls across fragments. Leaving
// layers clears cycle, but only WIPE_END clears wipe and a new export clears volume.
struct ClassificationState
{
    std::optional<CycleSnapshot> cycle;
    bool wipe_scope = false;
    bool volume_untrusted = false;
};

// One entry at the same zero-based physical row as SourceFragment.lines.
struct LineDecision
{
    // Close the previous pressure cycle before processing this row once; this
    // row can itself start a new cycle when it contains an eligible print.
    bool close_before = false;
    // Index into NominalFragment.motions only for eligible prints, not an event index.
    std::optional<std::size_t> motion_index;
};

// Owns exactly one decision per source row and final state to carry to the next
// fragment of the same export; contains no borrowed text or coordinate storage.
struct ClassificationFragment
{
    std::vector<LineDecision> lines;
    ClassificationState final;
};

// Identifies a missing printing proof or an unsupported active arc, volumetric
// extrusion or XYZ/unit mode. UnknownExtrusion covers required E values, not volume.
enum class ClassificationCause {
    UnknownTool, UnsupportedArc, UnsupportedVolume, UnknownModes,
    UnsupportedModes, UnknownCoordinates, UnknownExtrusion, UnknownFeedrate
};

// Reports a local physical row and proven active tool, without claiming a global line.
struct ClassificationError : std::invalid_argument
{
    // Local zero-based physical row, not the file's global line number.
    std::size_t line_index;
    // Proven active tool if known, including a value outside enabled_tools.
    std::optional<std::uint32_t> tool;
    // The specific failed proof or unsupported input reported in what().
    ClassificationCause cause;

    // Owns the local position and a readable explanation of the failed proof.
    ClassificationError(std::size_t line, std::optional<std::uint32_t> active_tool,
                        ClassificationCause reason);
};

// Returns one decision per physical row, with indices into nominal.motions only
// for eligible prints. Invalid hint sizes throw invalid_argument; broken motion
// joins or conflicting proven targets throw logic_error. Unsupported active input
// throws ClassificationError. No partial result escapes; copied prior is unchanged.
ClassificationFragment classify_fragment(const SourceFragment &source,
    const NominalFragment &nominal, const std::vector<MoveHint> &hints,
    const ClassificationContext &context, ClassificationState prior);

} // namespace nematx_pressure
