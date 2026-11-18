///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#ifndef slic3r_BridgeRows_hpp_
#define slic3r_BridgeRows_hpp_
/* Choose the pitch and the phase of the rows of a full-density bridge so that they cover its area best.
   BridgePreparation.cpp fits the rows when it prepares a bridge surface and stores them on it; its recipe
   modifier passes them to the rectilinear patterns, which place the rows.

   A row is one straight bridge thread. The pitch is the distance between two rows; bridge_overlap and
   bridge_overlap_min give its range as a density of the bridge flow spacing, and the nozzle diameter bounds it.
   The phase is the distance from the side of the rotated bounding box of the area to the first row, the
   recipe's bridge_offset. fit() reproduces how FillRectilinear keeps and extends rows, scores a row by the
   area it covers, a band one pitch wide, and searches the pitch and phase in two passes.

       const PitchRange range = pitch_range(config, spacing, nozzle_diameter);
       const Fit best = fit(storage, area, angle, spacing, range);
       const coord_t position = anchor(area, angle, best);
       // Later, on the final outline of the same bridge:
       const Fit same = anchored(final_area, angle, spacing, best.pitch, position);
       if (same.found()) { recipe.density = float(spacing / same.pitch); recipe.bridge_offset = same.phase; }

   Distances are in millimeters except the scaled phase; angles are in radians. Inputs stay unchanged and
   temporary geometry belongs to the caller's storage. */
#include <vector>
#include "libslic3r/Api/plugin/cpp/ClipperViews.hpp"
#include "libslic3r/Api/plugin/cpp/ConfigViews.hpp"
namespace slic3r_api { namespace BridgeRows {
// Allowed row pitches in millimeters, minimum <= maximum.
struct PitchRange { double minimum = 0., maximum = 0.; };
// The chosen rows: pitch in millimeters, scaled phase (-1 when no row fits), covered and overflowing area
// in fractions of the bridge area.
struct Fit {
    double pitch = 0.;
    coord_t phase = -1;
    double coverage = 0., overflow = 0.;
    bool found() const { return phase >= 0; }
};
// Pitch range of a bridge flow spacing from the region's bridge_overlap (densest rows) and bridge_overlap_min
// (sparsest rows); a minimum at or above the maximum density keeps the single bridge_overlap pitch. The pitch
// stays between half and twice the nozzle diameter.
PitchRange pitch_range(const Config &region_config, double spacing, double nozzle_diameter);
// Search the pitch and phase of the rows along angle that cover area best: ten pitches by ten phases, then
// five by five around the best with steps ten times finer, then the phase that centers the best rows. Equal
// coverages prefer less overflow, then the densest pitch, then the rows that leave the same space on both sides
// of the area, then the smallest phase. Only rows whose overflow is at most 1 - coverage are valid; no valid
// rows return a Fit that is not found().
Fit fit(storage_handle *storage, const ExPolygon &area, double angle, double spacing, const PitchRange &range);
// The absolute position, across the rows in the row frame of angle, of the first row of a found fit on area. Another
// outline of the same bridge gets these rows back from anchored().
coord_t anchor(const ExPolygon &area, double angle, const Fit &fit);
// The Fit that places rows at pitch through the absolute position anchor on area: same pitch, and the phase from the
// side of area's rotated bounding box, between zero and the pattern's line spacing. Coverage and overflow are not
// scored and stay zero; an invalid spacing or pitch gives a Fit that is not found().
Fit anchored(const ExPolygon &area, double angle, double spacing, double pitch, coord_t anchor);
// One predicted bridge thread: a straight segment between two scaled points of the area's frame.
struct Row { c_point from, to; };
// The rows that the rectilinear patterns draw on area for a found fit, as fit() models them: the line spacing that the
// recipe's density gives, the first line at fit.phase from the rotated bounding box, and every interval of the 0.05
// spacing contour that meets the 0.48 spacing contour. A fit that is not found() gives no row.
std::vector<Row> rows(storage_handle *storage, const ExPolygon &area, double angle, double spacing, const Fit &fit);
}}
#endif
