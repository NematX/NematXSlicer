///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
/* Pure tower layout uses millimeters and owns only value arrays, never a plan.
   Source layer groups are in print order, even when their Z decreases. A close
   group (its bottom overlaps the tower top) always shares the top tower layer;
   an equal or lower group shares it while the fused depth stays within the
   largest used depth so far, otherwise a layer is stacked above using the
   thinnest source layer's height.
   Bands reserve connected transverse lines using rounded Flow volume and spacing;
   suffix maxima make the tower no deeper toward its top, with no minimum depth.
   Sparse skipping looks ahead so a later printed layer remains within every
   involved tool limit.
   A floating band at or below the tower top is laid out with the other bands;
   above it, its source layer becomes one floating layer holding only that band,
   added after depth and cone so it changes neither; its own depth is its used
   depth. The cone base is calculated whenever the configured angle is positive. */
#ifndef nematx_wipetower_WipeTowerLayout_hpp_
#define nematx_wipetower_WipeTowerLayout_hpp_

#include <cstddef>
#include <vector>

namespace nematx_wipetower {
struct LayoutBandInput {
    size_t request_index;
    double volume;
    double line_width;
    double nozzle_diameter = 0.4;
    // True marks a band that may be printed above the tower top; it never makes the tower grow.
    bool floating = false;
};
struct LayoutLayerInput {
    double print_z;
    double height;
    double max_height;
    std::vector<LayoutBandInput> bands;
};
struct LayoutBand {
    size_t request_index;
    double y0;
    double y1;
    double line_width;
    bool enter_right;
    // Centre distance between two purge lines: rounded Flow spacing times the extra spacing ratio.
    double line_spacing = 0.;
};
struct LayoutLayer {
    size_t source_index;
    double print_z;
    double height;
    double used;
    // Depth shared by this layer and every layer above it: suffix maximum of used, with no minimum.
    double depth;
    std::vector<LayoutBand> bands;
    // A floating layer holds only floating bands and is never part of depth or cone; its depth is its own used depth.
    bool floating = false;
};
struct TowerLayout {
    std::vector<LayoutLayer> layers;
    double cone_base_radius = 0.;
    double cone_scale = 1.;
    double cone_slope = 0.;
};
// Reserve ordered bands, select printed layers, enforce suffix depth and calculate the cone base whenever the angle is positive; floating bands at or below the regular top are laid out with them, others get one floating layer each after depth and cone.
TowerLayout layout_tower(double width, double extra_spacing_ratio, bool no_sparse_layers,
                         double cone_angle, const std::vector<LayoutLayerInput> &layers);
// Calculate the cone base radius and horizontal support scaling using the legacy geometric formula.
void wipe_tower_cone_base(double width, double height, double depth, double angle,
                           double &radius, double &support_scale);
// Count whole X lines including the extruding connectors between successive lines.
size_t wipe_band_line_count(double volume, double width, double mm3_per_mm, double pitch);
}
#endif
