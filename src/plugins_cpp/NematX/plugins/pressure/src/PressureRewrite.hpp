///|/ Released under the GNU Affero General Public License, version 3 or later.

// Replaces one already eligible printing row with prepared spatial groups.
// Source/Nominal must come from the same Reader call and export; Geometry and
// Extrusion results must already be aligned in emission order. No calculation
// of phases, pressure, configuration or inter-source merging happens here.
// For example, after preparing those values:
//   std::string bytes = rewrite_print(source, row, motion, geometry, extrusion,
//                                     k, axes, {3, 5, true});
// The returned owned string replaces the whole row, retaining its original
// terminator when present; otherwise only inter-group rows receive LF.
// For a source whose material has been assigned to another movement, call
// rewrite_residue instead; it removes spatial and extrusion words, not material.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace nematx_pressure {
struct SourceFragment;
struct NominalMotion;
struct GeometryResult;
struct ExtrusionResult;

// Axis precision is 0..9. annotate enables one NX_PRESSURE annotation per
// emitted group, with round-trip precision independent of the axis digits.
struct RewriteFormat { int xyz_digits; int e_digits; bool annotate; };

// These causes apply only to eligible printing rows passed to either rewriter.
enum class RewriteCause { InvalidBytes, DuplicateParameter, ConflictingXyzModes, UnsupportedWord, LostModalEffect };

// Owns a local physical row, proven active tool and explicit source failure.
struct RewriteError : std::invalid_argument
{
    // Zero-based physical row within the fragment, not the whole output file.
    std::size_t line_index;
    // Proven active tool, not an extrusion target inferred from a source letter.
    std::uint32_t tool;
    // The ambiguity, unsupported word or lost effect described by what().
    RewriteCause cause;

    // Builds a diagnostic identifying the local row, tool and rejected source.
    RewriteError(std::size_t row, std::uint32_t active_tool, RewriteCause reason);
};

// Returns all replacement bytes only on success, without changing borrowed inputs.
// Requires a proven LINEAR_MOVE in this row, absolute mm XYZ, known E mode/tool,
// representable Geometry count 1..3 and matching numeric source spans. Missing E
// is allowed only for producer-proven printing; insertion requires exact G0/G1.
// Invalid prepared shapes throw invalid_argument or logic_error; ambiguous or
// unsupported source words/bytes throw RewriteError. No partial result escapes.
std::string rewrite_print(const SourceFragment &source, std::size_t row,
    const NominalMotion &motion, const GeometryResult &geometry,
    const std::array<ExtrusionResult, 3> &extrusion, double pressure_factor_s,
    const std::vector<char> &resolved_axis_letters, const RewriteFormat &format);

// Removes complete XYZ/E words from a proven linear mm row with absolute XYZ,
// known E mode/tool, finite nonnegative nominal E and matching source spans.
// The caller must arrange material transfer before using this owned result.
// Keeps original bytes outside removed words when F or a modal effect remains;
// otherwise returns the comment with its original ending, or an empty string.
// Throws RewriteError if lexical ambiguity or loss of a mode/tool effect would
// occur, and invalid_argument/logic_error for invalid prepared inputs. Inputs
// remain unchanged and no partial result or pressure annotation is returned.
std::string rewrite_residue(const SourceFragment &source, std::size_t row,
    const NominalMotion &motion, const std::vector<char> &resolved_axis_letters);

} // namespace nematx_pressure
